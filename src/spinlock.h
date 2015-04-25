#ifndef SPINLOCK_H
#define SPINLOCK_H
typedef volatile int spinlock;
void spin_lock(spinlock *lock);
void spin_unlock(spinlock *lock);
#endif
