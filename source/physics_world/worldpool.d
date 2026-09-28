module physics_world.worldpool;

import std.parallelism : totalCPUs;
import std.algorithm : max, min;
import core.sync.mutex : Mutex;
import core.sync.condition : Condition;

import physics_world.engine;

/**
 * Пул физических миров Newton для генетического драйвера.
 *
 * Вместо того чтобы создавать и разрушать `NewtonPhysicsWorld` на каждого
 * индивида (create + step + dispose в каждом заезде), миры переиспользуются.
 *
 * ПОЧЕМУ ТАК, а не «создать/уничтожить»:
 * У Newton 3.14 (предсобранные libnewton.so/libdgCore.so) есть дефект: при
 * быстром create/destroy миров внутренние потоки мира (dgMutexThread /
 * dgAsyncThread) могут не успеть завершиться, память мира переиспользуется
 * новым экземпляром, и на одном объекте живут два исполняющихся потока.
 * Тогда `dgMutexThread::Terminate()` будит семафор только один раз, а `join()`
 * ждёт конкретный поток — который остаётся в `Wait()` навсегда. Итог: полный
 * фриз фитнес-прохода (`NewtonDestroy` → `std::thread::join` висит вечно),
 * воркеры ген. пула копят LVР-потоки (16 → 36 → 60), поколение застревает.
 *
 * Рабочий обход БЕЗ правки dagon/Newton: мир из пула не уничтожается
 * (`NewtonDestroy` не вызывается), перед возвратом в пул он очищается только
 * от тел (`NewtonDestroyAllBodies`). Внутренние потоки мира живут всё время
 * жизни пула, плодить новые потоки незачем, и дефектный путь create/destroy
 * из горячего цикла уходит. Цена — до `capacity` физических миров (и их
 * внутренних потоков) на процесс.
 *
 * Когда Newton исправят дефект (гарантия погашения всех внутренних потоков
 * мира при `NewtonDestroy`), пул можно убрать и вернуться к простому
 * create/step/dispose на каждого индивида: `BuggyPhysics` сам создаёт мир.
 */
final class PhysWorldPool
{
    private Mutex lock_;
    private Condition cond_;

    /// Свободные миры, готовые к выгрузке следующего заезда.
    private PhysWorld[] free_;
    /// Сколько миров создано всего (никогда не превышает `capacity_`).
    private size_t total_;
    private immutable size_t capacity_;

    this(size_t capacity)
    {
        lock_ = new Mutex();
        cond_ = new Condition(lock_);
        capacity_ = max(capacity, 1);
    }

    /// Взять мир из пула (блокирует, если все миры заняты и пул полон).
    /// Мир приходит ПУСТЫМ (без тел): модель строится заново.
    PhysWorld acquire()
    {
        synchronized (lock_)
        {
            while (free_.length == 0 && total_ >= capacity_)
                cond_.wait();
            PhysWorld w;
            if (free_.length)
            {
                w = free_[$ - 1];
                free_.length--;
            }
            else
            {
                w = createWorld();
                total_++;
            }
            assert(w !is null);
            return w;
        }
    }

    /// Вернуть мир в пул: уничтожить все тела заезда, но НЕ сам мир.
    /// Группы материалов и колбэки мира живут — следующая модель
    /// переиспользует `sensorGroupId`/`defaultGroupId` как есть.
    void release(PhysWorld w)
    {
        synchronized (lock_)
        {
            // Только тела: группы материалов и колбэки остаются, следующий
            // заезд переиспользует их как есть.
            w.clearScene();
            free_ ~= w;
            cond_.notifyAll();
        }
    }

    private static PhysWorld createWorld()
    {
        auto w = createPhysWorld();
        w.useCallingThread();
        return w;
    }
}

/// Потолок миров в пуле. Каждый мир — целый Newton со своими телами и сеткой
/// земли 193x193, и на 16 ядрах прежние 12 миров дергали всю машину, не
/// оставляя системе ничего, кроме себя.
enum size_t maxPooledWorlds = 4;

/// Один глобальный пул на процесс: физику гоняют и воркеры TaskPool
/// (до 75% ядер), и главный поток вьюера. Мир из пула живёт столько,
/// сколько нужно; число миров — ровно степень параллелизма, не больше.
/// Инициализируется под __gshared-локом: первый дозвавшийся поток строит пул.
private __gshared Object poolLock_ = new Object();
private __gshared PhysWorldPool pool_;

/// Сколько миров в пуле на этой машине. Нижняя граница — 2: один мир держит
/// живой заезд вьюера на реальном времени, и воркерам при одном мире ждать.
size_t pooledWorldCount()
{
    return min(maxPooledWorlds, max(2, (cast(size_t) totalCPUs * 3) / 4));
}

private PhysWorldPool worldPool()
{
    if (pool_ !is null)
        return pool_;
    synchronized (poolLock_)
    {
        if (pool_ is null)
            pool_ = new PhysWorldPool(pooledWorldCount());
        return pool_;
    }
}

/// Взять мир для одного заезда. Парный вызов — `releaseWorld`.
PhysWorld acquireWorld()
{
    return worldPool().acquire();
}

/// Вернуть мир после заезда (очищенный от тел, без `NewtonDestroy`).
void releaseWorld(PhysWorld w)
{
    worldPool().release(w);
}