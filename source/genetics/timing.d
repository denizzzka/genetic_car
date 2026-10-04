module genetics.timing;

import std.format : format;

/// Замеры фаз поколения, с: рост, отбор и физика. Фронтенды наполняют
/// структуру по-своему (headless считает дельты одного цикла, вьюер —
/// свои секундомеры), а строку печатает toPhases: формат один на всех.
struct GenerationTiming
{
    double grow;
    double eval;
    double physics;

    @property double cycle() const
    {
        return grow + eval + physics;
    }

    string toPhases() const
    {
        return format("рост=%.2fs отбор=%.2fs физика=%.2fs цикл=%.2fs",
            grow, eval, physics, cycle);
    }
}