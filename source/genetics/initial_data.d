module genetics.initial_data;

// Стартовая хромосома эволюции — откалиброванный основатель вида.

import genetics.chromosome : Chromosome, alleles;

/**
 * Хромосома основателя: значения по умолчанию `Chromosome`. С ними химия роста
 * даёт машину, которая уже едет, — нулевое поколение не тратит эволюцию на
 * поиск формы с нуля.
 */
Chromosome startChromosome()
{
    return Chromosome.init;
}

unittest
{
    import genetics.growth : develop;

    // Основатель — полный набор генов и годная машина: поколение 0 не должно
    // проваливаться в ноль фитнеса.
    const auto founder = startChromosome;
    assert(founder.alleles == Chromosome.init.alleles,
        "основатель — это значения по умолчанию хромосомы");

    const auto grown = develop(founder);
    assert(!grown.isNull, "основатель обязан вырасти в машину");
}