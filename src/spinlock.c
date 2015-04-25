#include "spinlock.h"
void spin_lock(spinlock *lock)
{
    while (__sync_lock_test_and_set(lock, 1))
        ;
}

void spin_unlock(spinlock *lock)
{
    __sync_lock_release(lock);
}
