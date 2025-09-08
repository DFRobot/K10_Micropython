#ifndef CLEANUP_H
#define CLEANUP_H
void cleanup_all_resources(void);
void mp_raise_type_with_cleanup(const mp_obj_type_t *exc_type);
#endif 