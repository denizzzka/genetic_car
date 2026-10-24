module physics_world.retry;

import std.algorithm : max;

import physics_world.physics : RunOutcome;

/// Сходы, которые имеет смысл лечить сбросом газа.
bool retryable(RunOutcome why)
{
    return why == RunOutcome.beamGround || why == RunOutcome.cabinGround;
}

/**
 * Политика отката при сходе. `T` - время последнего схода.
 *
 * Первая попытка берёт снимок не позже `T - lead` и отпускает газ с
 * `T - lead` до `T + hold`; следующая берёт снимок старее и отпускает газ
 * от него до `T + hold`, чтобы машина подошла медленнее. Успех - `T +
 * successAfter` без нового схода; иначе следующая попытка, пока есть снимки
 * старее. Кончились - сход остаётся окончательным.
 */
struct RetryState
{
    double lead = 1.0;
    double hold = 1.0;
    double successAfter = 2.0;

    bool active;
    uint attempts;
    double failTime;
    double releaseStart;
    double releaseEnd;

    private double cursor_;
    private bool first_;

    /// Зафиксировать сход. Первый раз задаёт `T` и начало поиска снимка;
    /// повторный сход обновляет `T`, но поиск продолжает уходить старее.
    void beginFailure(double now)
    {
        if (!active)
        {
            active = true;
            attempts = 0;
            failTime = now;
            cursor_ = now - lead;
            first_ = true;
        }
        else
        {
            failTime = now;
            first_ = false;
        }
    }

    /// Выбрать снимок и окно сброса газа. Восстановление мира и внешнего
    /// состояния делает вызывающий: у живого заезда и фитнеса они разные.
    bool select(H, T)(ref H history, out T* snapshot,
        out double releaseStart, out double releaseEnd)
    {
        static assert(is(H.Value == T),
            "снимок должен быть типом значения истории");
        auto snap = history.latestBefore(cursor_);
        if (snap is null && first_)
            snap = history.oldest();
        if (snap is null)
        {
            active = false;
            return false;
        }

        const double snapTime = snap.time;
        releaseStart = first_ ? max(snapTime, failTime - lead) : snapTime;
        releaseEnd = failTime + hold;
        cursor_ = snapTime - 1e-6;
        this.releaseStart = releaseStart;
        this.releaseEnd = releaseEnd;
        ++attempts;
        snapshot = cast(T*) snap;
        return true;
    }

    /// Газ на попытке: 0 в окне сброса, иначе полный.
    float throttle(double time) const
    {
        if (!active)
            return 1.0f;
        return (time < releaseStart || time >= releaseEnd) ? 1.0f : 0.0f;
    }

    /// Откат пережил окно сброса без нового схода.
    bool succeeded(double time) const
    {
        return active && time > failTime + successAfter;
    }

    void reset()
    {
        active = false;
        attempts = 0;
        failTime = 0.0;
        releaseStart = 0.0;
        releaseEnd = 0.0;
        cursor_ = 0.0;
        first_ = false;
    }
}

unittest
{
    import physics_world.history : History;

    assert(retryable(RunOutcome.beamGround));
    assert(retryable(RunOutcome.cabinGround));
    assert(!retryable(RunOutcome.offCourse));
    assert(!retryable(RunOutcome.frameBroke));

    struct Slot
    {
        double time;
        int value;
    }

    History!Slot history;
    history.interval = 5.0;
    history.span = 30.0;
    foreach (i; 0 .. 4)
        history.capture(Slot(5.0 * i, cast(int) (5.0 * i)), 5.0 * i);

    RetryState r;
    r.beginFailure(12.0);
    Slot* snap;
    double releaseStart, releaseEnd;
    assert(r.select(history, snap, releaseStart, releaseEnd));
    assert(snap.value == 10, "первый откат - снимок не позже T-1");
    assert(releaseStart == 11.0 && releaseEnd == 13.0);
    assert(r.attempts == 1);
    assert(r.throttle(10.5) == 1.0f);
    assert(r.throttle(11.5) == 0.0f);
    assert(r.throttle(13.0) == 1.0f);
    assert(!r.succeeded(13.9));
    assert(r.succeeded(14.1));

    r.beginFailure(12.5);
    assert(r.select(history, snap, releaseStart, releaseEnd));
    assert(snap.value == 5, "следующий откат уходит старее");
    assert(releaseStart == 5.0 && releaseEnd == 13.5);
    assert(r.attempts == 2);

    r.beginFailure(13.0);
    assert(r.select(history, snap, releaseStart, releaseEnd));
    assert(snap.value == 0);
    assert(releaseStart == 0.0 && releaseEnd == 14.0);

    r.beginFailure(14.0);
    assert(!r.select(history, snap, releaseStart, releaseEnd));
    assert(!r.active, "снимков старше нет - откат заканчивается");

    r.reset();
    assert(!r.active && r.attempts == 0);
}
