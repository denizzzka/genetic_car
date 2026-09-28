/**
 * Журнал контактов за шаг симуляции.
 *
 * Общий для всех бэкендов: и колбэки C++ Newton, и контакт-листендер Jolt
 * пишут пары тел сюда, а читает их модель через `PhysWorld.contacts`.
 *
 * Живёт отдельно от мира, чтобы колбэкам не нужно было знать тип мира: они
 * приходят из движка и видят только указатели на тела.
 *
 * Хранится ПО ЗНАЧЕНИЮ в мире и держит буфер пар в dlib-памяти. Пока журнал
 * был GC-классом, единственные ссылки на него жили в сырых указателях движка
 * (мир и сенсорные тела), а GC такую память не обходит: под давлением
 * аллокаций журнал уходил в free-list и переписывался чужими данными прямо
 * во время заезда — зависание и SIGSEGV в разборе контактов.
 */
module physics_world.contactlog;

import dlib.core.memory;

import physics_world.engine : PhysBody, ContactPair;

/// Буфер пар в dlib-памяти: GC-объекты из мира его не увидят.
struct ContactLog
{
    private ContactPair[] buf_;
    private size_t len_;

    const(ContactPair)[] pairs() const { return buf_[0 .. len_]; }

    /// Повторы гасим: сенсорный диспетчер и материальный колбэк могут
    /// сообщить одну пару дважды, а вердикту важно лишь «было». Контактов на
    /// шаг единицы, поэтому поиск линейный.
    void add(PhysBody a, PhysBody b)
    {
        if (a is null || b is null || a is b)
            return;
        foreach (i; 0 .. len_)
            if (buf_[i].a is a && buf_[i].b is b)
                return;
        if (len_ == buf_.length)
            grow();
        buf_[len_++] = ContactPair(a, b);
    }

    void clear()
    {
        len_ = 0;
    }

    void dispose()
    {
        freeBuf();
        len_ = 0;
    }

    private void grow()
    {
        const size_t cap = buf_.length == 0 ? 16 : buf_.length * 2;
        auto grown = New!(ContactPair[])(cap);
        foreach (i; 0 .. len_)
            grown[i] = buf_[i];
        freeBuf();
        buf_ = grown;
    }

    /// `Delete` у dlib читает заголовок размера перед указателем, поэтому
    /// на пустом массиве (контактов не было) он падает.
    private void freeBuf()
    {
        if (buf_ !is null)
            Delete(buf_);
        buf_ = null;
    }
}
