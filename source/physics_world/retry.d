module physics_world.retry;

import std.algorithm : max;

import physics_world.physics : RunOutcome, baseThrottle;

/// Газ в окне, когда машину учат выбираться из застоя.
enum float gasThrottle = 1.0f;

/// Сходы, которые имеет смысл лечить откатом.
bool retryable(RunOutcome why)
{
    return why == RunOutcome.beamGround || why == RunOutcome.cabinGround
        || why == RunOutcome.stalled;
}

/// Что делает откат в своём окне.
enum RetryAction
{
    /// Касание земли: отпустить газ.
    release,
    /// Застой: дать полный газ.
    gas,
}

/**
 * Политика отката при сходе. `T` - время последнего схода.
 *
 * Первая попытка берёт снимок не позже `T - lead` и меняет газ с
 * `T - lead` до `T + hold`: при касании земли отпускает до нуля, при застое
 * даёт полный. Следующая попытка берёт снимок старее и держит окно от него,
 * чтобы машина подошла медленнее или успела разогнаться. Успех - `T +
 * successAfter` без нового схода; иначе следующая попытка, пока есть снимки
 * старее. Кончились - сход остаётся окончательным.
 */
struct RetryState
{
    double lead = 1.0;
    /// Шаг отката при застое: сначала T-5, потом T-10 и так до окна истории.
    double stallLead = 5.0;
    double hold = 1.0;
    double successAfter = 2.0;

    bool active;
    uint attempts;
    double failTime;
    double releaseStart;
    double releaseEnd;
    RetryAction action;

    private double lead_;
    private double cursor_;
    private bool first_;

    /// Зафиксировать сход. Первый раз задаёт `T` и начало поиска снимка;
    /// повторный сход обновляет `T`, но поиск продолжает уходить старее.
    void beginFailure(RunOutcome why, double now)
    {
        action = why == RunOutcome.stalled ? RetryAction.gas
            : RetryAction.release;
        lead_ = why == RunOutcome.stalled ? stallLead : lead;
        if (!active)
        {
            active = true;
            attempts = 0;
            failTime = now;
            cursor_ = now - lead_;
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
        releaseStart = first_ ? max(snapTime, failTime - lead_) : snapTime;
        releaseEnd = failTime + hold;
        cursor_ = snapTime - 1e-6;
        this.releaseStart = releaseStart;
        this.releaseEnd = releaseEnd;
        ++attempts;
        snapshot = cast(T*) snap;
        return true;
    }

    /// Газ на попытке: в окне действия 0 для касания и 1.0 для застоя,
    /// вне окна - базовый.
    float throttle(double time) const
    {
        if (!active)
            return baseThrottle;
        if (time < releaseStart || time >= releaseEnd)
            return baseThrottle;
        return action == RetryAction.gas ? gasThrottle : 0.0f;
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
        action = RetryAction.release;
        lead_ = 0.0;
        cursor_ = 0.0;
        first_ = false;
    }
}

unittest
{
    import physics_world.history : History;

    assert(retryable(RunOutcome.beamGround));
    assert(retryable(RunOutcome.cabinGround));
    assert(retryable(RunOutcome.stalled));
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
    r.beginFailure(RunOutcome.beamGround, 12.0);
    Slot* snap;
    double releaseStart, releaseEnd;
    assert(r.select(history, snap, releaseStart, releaseEnd));
    assert(snap.value == 10, "первый откат - снимок не позже T-1");
    assert(releaseStart == 11.0 && releaseEnd == 13.0);
    assert(r.attempts == 1);
    assert(r.throttle(10.5) == baseThrottle);
    assert(r.throttle(11.5) == 0.0f);
    assert(r.throttle(13.0) == baseThrottle);
    assert(!r.succeeded(13.9));
    assert(r.succeeded(14.1));

    r.beginFailure(RunOutcome.beamGround, 12.5);
    assert(r.select(history, snap, releaseStart, releaseEnd));
    assert(snap.value == 5, "следующий откат уходит старее");
    assert(releaseStart == 5.0 && releaseEnd == 13.5);
    assert(r.attempts == 2);

    r.beginFailure(RunOutcome.beamGround, 13.0);
    assert(r.select(history, snap, releaseStart, releaseEnd));
    assert(snap.value == 0);
    assert(releaseStart == 0.0 && releaseEnd == 14.0);

    r.beginFailure(RunOutcome.beamGround, 14.0);
    assert(!r.select(history, snap, releaseStart, releaseEnd));
    assert(!r.active, "снимков старше нет - откат заканчивается");

    r.reset();
    assert(!r.active && r.attempts == 0);

    // Застой лечится газом: в окне 1.0, вне окна базовый 0.7.
    RetryState gas;
    gas.beginFailure(RunOutcome.stalled, 12.0);
    assert(gas.select(history, snap, releaseStart, releaseEnd));
    assert(snap.value == 5, "первый откат застоя - на 5 секунд назад");
    assert(gas.action == RetryAction.gas);
    assert(releaseStart == 7.0 && releaseEnd == 13.0);
    assert(gas.throttle(0.5) == baseThrottle);
    assert(gas.throttle(7.5) == gasThrottle);
    assert(gas.throttle(12.5) == gasThrottle);
    assert(gas.throttle(13.0) == baseThrottle);

    // Следующий сход уходит ещё на 5 секунд назад, пока есть история.
    gas.beginFailure(RunOutcome.stalled, 13.0);
    assert(gas.select(history, snap, releaseStart, releaseEnd));
    assert(snap.value == 0, "второй откат застоя - ещё на 5 секунд назад");
}
