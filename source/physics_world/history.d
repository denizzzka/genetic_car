module physics_world.history;

import physics_world.car : BuggyPhysics;
import physics_world.state : WorldSnapshot;

/**
 * Кольцевой буфер снимков мира: раз в `interval` секунд, хранит `span`.
 *
 * Снимки ставятся вызовом `maybeCapture` после шага; сам мир ничего о буфере
 * не знает, поэтому фитнес может его не заводить. Память ограничена
 * `capacity` слотами.
 */
struct SnapshotHistory
{
    /// Как часто снимать мир, с.
    double interval = 5.0;

    /// Сколько секунд истории хранить, с.
    double span = 30.0;

    private WorldSnapshot[] slots_;
    private double[] times_;
    private size_t head_;
    private size_t count_;
    private double nextTime_;
    private bool started_;

    /// Сколько снимков помещается в окно.
    size_t capacity() const
    {
        if (interval <= 0.0)
            return 1;
        const size_t n = cast(size_t) (span / interval) + 1;
        return n > 0 ? n : 1;
    }

    size_t length() const @property
    {
        return count_;
    }

    bool empty() const @property
    {
        return count_ == 0;
    }

    /// Время старейшего снимка; NaN, если буфер пуст.
    double oldestTime() const @property
    {
        return count_ == 0 ? double.nan : times_[head_];
    }

    /// Время новейшего снимка; NaN, если буфер пуст.
    double newestTime() const @property
    {
        return count_ == 0 ? double.nan
            : times_[(head_ + count_ - 1) % slots_.length];
    }

    /// Снять мир, если прошёл `interval` с прошлого снимка.
    bool maybeCapture(BuggyPhysics physics)
    {
        if (started_ && physics.time < nextTime_)
            return false;
        capture(physics);
        return true;
    }

    /// Снять мир сейчас, не дожидаясь интервала.
    void capture(BuggyPhysics physics)
    {
        ensureStorage();
        const double now = physics.time;
        const size_t idx = (head_ + count_) % slots_.length;
        slots_[idx] = physics.saveState();
        times_[idx] = now;
        if (count_ < slots_.length)
            ++count_;
        else
            head_ = (head_ + 1) % slots_.length;
        nextTime_ = now + interval;
        started_ = true;

        while (count_ > 0 && times_[head_] < now - span)
        {
            head_ = (head_ + 1) % slots_.length;
            --count_;
        }
    }

    /// Новейший снимок.
    const(WorldSnapshot)* latest() const
    {
        if (count_ == 0)
            return null;
        return &slots_[(head_ + count_ - 1) % slots_.length];
    }

    /// Самый старый снимок.
    const(WorldSnapshot)* oldest() const
    {
        return count_ == 0 ? null : &slots_[head_];
    }

    /// Выбросить снимки новее `time`: после отката они из провалившейся ветки.
    void truncateAfter(double time)
    {
        while (count_ > 0)
        {
            const size_t idx = (head_ + count_ - 1) % slots_.length;
            if (times_[idx] <= time)
                break;
            --count_;
        }
        nextTime_ = time + interval;
        started_ = count_ > 0;
    }

    /// Новейший снимок не позже `time`.
    const(WorldSnapshot)* latestBefore(double time) const
    {
        for (size_t k = count_; k > 0; --k)
        {
            const size_t idx = (head_ + k - 1) % slots_.length;
            if (times_[idx] <= time)
                return &slots_[idx];
        }
        return null;
    }

    /// Откатить мир к новейшему снимку. `false` — снимков нет.
    bool rewind(BuggyPhysics physics)
    {
        const s = latest();
        if (s is null)
            return false;
        physics.restoreState(*s);
        return true;
    }

    /// Откатить мир к снимку не позже `time`.
    bool rewindBefore(BuggyPhysics physics, double time)
    {
        const s = latestBefore(time);
        if (s is null)
            return false;
        physics.restoreState(*s);
        return true;
    }

    void clear()
    {
        slots_ = null;
        times_ = null;
        head_ = 0;
        count_ = 0;
        nextTime_ = 0.0;
        started_ = false;
    }

    private void ensureStorage()
    {
        const size_t cap = capacity;
        if (slots_.length == cap)
            return;
        slots_ = new WorldSnapshot[cap];
        times_ = new double[cap];
        head_ = 0;
        count_ = 0;
    }
}

unittest
{
    import physics_world.car : Buggy;
    import physics_world.physics : placedFrame;
    import genetics.growth : develop;
    import genetics.initial_data : startChromosome;

    const auto grown = develop(startChromosome());
    assert(!grown.isNull, "основатель должен вырасти");
    auto physics = new BuggyPhysics(new Buggy(placedFrame(grown.get)));
    scope (exit) physics.dispose();

    const double dt = 1.0 / 60.0;
    physics.settle(dt, 30);

    SnapshotHistory h;
    h.maybeCapture(physics);
    foreach (_; 0 .. 7 * 300 + 30)
    {
        physics.step(dt, 1.0f);
        h.maybeCapture(physics);
    }

    assert(h.length <= h.capacity, "буфер не должен расти сверх окна");
    assert(h.length >= 6, "за 35 секунд должно накопиться несколько снимков");
    assert(h.oldestTime >= physics.time - h.span - 1e-6,
        "старые снимки должны выпадать из окна");

    const auto latest = h.latest();
    assert(latest !is null, "после заезда есть новейший снимок");
    const double latestTime = latest.time;
    assert(h.rewind(physics), "откат к новейшему снимку");
    assert(physics.time == latestTime, "откат возвращает время снимка");

    const auto older = h.latestBefore(physics.time - 10.0);
    if (older !is null)
    {
        assert(h.rewindBefore(physics, physics.time - 10.0));
        assert(physics.time == older.time, "откат к снимку не позже времени");

        const double olderTime = older.time;
        h.truncateAfter(olderTime);
        assert(h.newestTime == olderTime, "обрезка оставляет выбранный снимок");
        assert(h.length >= 1, "после обрезки буфер не пуст");
    }
}
