#ifndef LINESEARCH_H
#define LINESEARCH_H


//Deklaracija funkcija definisanih u LineSearch.cpp

struct Variable line (struct Variable point, struct Variable direction, double alpha);

double derivative_on_line (double( *f)(struct Variable, struct Variable, struct Variable),
                           struct Variable dh, struct Variable dpc, struct Variable point,
                           struct Variable direction, double t);

double second_derivative (double( *f)(struct Variable, struct Variable, struct Variable),
                          struct Variable dh, struct Variable dpc, struct Variable point,
                          struct Variable direction, double t);

struct Variable newton_two_points (double( *f)(struct Variable, struct Variable, struct Variable), 
                                   struct Variable dh, struct Variable dpc, struct Variable point,
                                   struct Variable direction);

#endif
