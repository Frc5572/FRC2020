#ifndef PATHFINDER_ERROR_H_DEF
#define PATHFINDER_ERROR_H_DEF


char * pathfinder_error();
void pathfinder_set_error(const char *msg);

void pathfinder_clear_errors();
int pathfinder_has_error();

#endif