module physics_world.history;

import physics_world.car : BuggyPhysics;
import physics_world.state : WorldSnapshot;

/**
 * Кольцевой буфер снимков: раз в `interval` секунд, хранит `span`.
 *
 * Значение и время кладёт вызывающий, поэтому буфер годится и для мира, и для
 * снимка заезда с внешним прогрессом. Память ограничена `capacity` слотами.
 */
struct History(T)
{
    /// Как часто снимать, с.
    double interval = 5.0;

    /// Сколько секунд истории хранить, с.
    double span = 30.0;

    alias Value = T;

    private T[] slots_;
    private double[] times_;
    private size_t head_;
    private size_t count_;
    private double nextTime_;
    private bool started_;

    /// Сколько снимков помещается в окно.
    size_t capacity() const @property
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

    /// Пора ли делать следующий снимок.
    bool due(double time) const
    {
        return !started_ || time >= nextTime_;
    }

    /// Снять значение, если прошёл `interval`.
    bool maybeCapture(T value, double time)
    {
        if (!due(time))
            return false;
        capture(value, time);
        return true;
    }

    /// Снять значение сейчас, не дожидаясь интервала.
    void capture(T value, double time)
    {
        ensureStorage();
        const size_t idx = (head_ + count_) % slots_.length;
        slots_[idx] = value;
        times_[idx] = time;
        if (count_ < slots_.length)
            ++count_;
        else
            head_ = (head_ + 1) % slots_.length;
        nextTime_ = time + interval;
        started_ = true;

        while (count_ > 0 && times_[head_] < time - span)
        {
            head_ = (head_ + 1) % slots_.length;
            --count_;
        }
    }

    /// Новейший снимок.
    const(T)* latest() const
    {
        if (count_ == 0)
            return null;
        return &slots_[(head_ + count_ - 1) % slots_.length];
    }

    /// Самый старый снимок.
    const(T)* oldest() const
    {
        return count_ == 0 ? null : &slots_[head_];
    }

    /// Новейший снимок не позже `time`.
    const(T)* latestBefore(double time) const
    {
        for (size_t k = count_; k > 0; --k)
        {
            const size_t idx = (head_ + k - 1) % slots_.length;
            if (times_[idx] <= time)
                return &slots_[idx];
        }
        return null;
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
        slots_ = new T[cap];
        times_ = new double[cap];
        head_ = 0;
        count_ = 0;
    }
}

/// Буфер снимков мира багги: удобные методы поверх `History!WorldSnapshot`.
struct SnapshotHistory
{
    private History!WorldSnapshot hist_;

    alias Value = WorldSnapshot;

    @property double interval() const { return hist_.interval; }
    @property void interval(double v) { hist_.interval = v; }
    @property double span() const { return hist_.span; }
    @property void span(double v) { hist_.span = v; }
    @property size_t capacity() const { return hist_.capacity; }
    @property size_t length() const { return hist_.length; }
    @property bool empty() const { return hist_.empty; }
    @property double oldestTime() const { return hist_.oldestTime; }
    @property double newestTime() const { return hist_.newestTime; }

    bool due(double time) const { return hist_.due(time); }

    bool maybeCapture(BuggyPhysics physics)
    {
        if (!hist_.due(physics.time))
            return false;
        hist_.capture(physics.saveState(), physics.time);
        return true;
    }

    void capture(BuggyPhysics physics)
    {
        hist_.capture(physics.saveState(), physics.time);
    }

    const(WorldSnapshot)* latest() const { return hist_.latest(); }
    const(WorldSnapshot)* oldest() const { return hist_.oldest(); }
    const(WorldSnapshot)* latestBefore(double time) const
    {
        return hist_.latestBefore(time);
    }

    bool rewind(BuggyPhysics physics)
    {
        const s = latest();
        if (s is null)
            return false;
        physics.restoreState(*s);
        return true;
    }

    bool rewindBefore(BuggyPhysics physics, double time)
    {
        const s = latestBefore(time);
        if (s is null)
            return false;
        physics.restoreState(*s);
        return true;
    }

    void truncateAfter(double time) { hist_.truncateAfter(time); }
    void clear() { hist_.clear(); }
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
