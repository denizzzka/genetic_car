/**
 * Выбор физического движка.
 *
 * Модель машины не знает про движки: она просит `createPhysWorld` и получает
 * `PhysWorld`. Кто за этим стоит — Newton или Jolt — решает здесь, один раз на
 * процесс, до того как вьюер и генетический драйвер создадут первые миры.
 *
 * Выбор приходит из командной строки (`--engine jolt|newton`) и разбирается
 * ДО старта dagon: dagon свои аргументы командной строки не разбирает, но
 * лишний флаг в `args` вьюеру лучше не отдавать.
 */
module physics_world.engineselect;

import std.algorithm : startsWith;
import std.array : array;
import std.exception : enforce;

import physics_world.engine;
import physics_world.engine_jolt;
import physics_world.engine_newton;

/// Выбранный движок. Читается после `selectEngine`; до него — `jolt`: Jolt
/// выбран движком по умолчанию.
private __gshared PhysicsEngine selected_ = PhysicsEngine.jolt;

enum PhysicsEngine
{
    newton,
    jolt,
}

/// Имя движка для вывода и разбора командной строки: `jolt`/`newton`.
string engineName(PhysicsEngine e) @property
{
    final switch (e)
    {
        case PhysicsEngine.newton: return "newton";
        case PhysicsEngine.jolt: return "jolt";
    }
}

PhysicsEngine selectedEngine() @property
{
    return selected_;
}

/// Разобрать `--engine <имя>` (или `--engine=<имя>`) и вернуть args без
/// разобранного флага: лишние аргументы вьюеру dagon передавать не нужно.
/// Неизвестное имя — исключение, а не молчаливый откат: иначе прогон, который
/// просили вести на Jolt, тихо уехал бы на Newton.
string[] selectEngine(string[] args)
{
    size_t i;
    for (; i < args.length; ++i)
    {
        const string arg = args[i];
        string name;
        bool inline = false;
        if (arg == "--engine" || arg == "-engine")
        {
            enforce(i + 1 < args.length, "--engine без имени движка");
            name = args[i + 1];
            ++i;
        }
        else if (arg.startsWith("--engine="))
        {
            name = arg["--engine=".length .. $];
            inline = true;
        }
        else
            continue;
        enforce(name == "jolt" || name == "newton",
            "неизвестный движок: " ~ name ~ " (доступны jolt, newton)");
        if (name == "jolt")
            selected_ = PhysicsEngine.jolt;
        else
            selected_ = PhysicsEngine.newton;
        return (args[0 .. i - (inline ? 0 : 1)]
            ~ args[(inline ? i + 1 : i + 2) .. $]).array;
    }
    return args;
}

/// Имя выбранного движка: для вывода в прологе прогона.
string selectedEngineName() @property
{
    return engineName(selected_);
}

/// Мир выбранного движка — единственное место в коде, где бэкенд выбирается.
PhysWorld createPhysWorld()
{
    final switch (selected_)
    {
        case PhysicsEngine.newton:
            return physics_world.engine_newton.createPhysWorld();
        case PhysicsEngine.jolt:
            return physics_world.engine_jolt.createPhysWorld();
    }
}

/// Умеет ли движок рулевую балку. У Jolt рулевой шарнир — заглушка, поэтому
/// тесты руля на нём не имеют смысла.
bool engineHasSteerJoint() @property
{
    return selected_ == PhysicsEngine.jolt;
}
