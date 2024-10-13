#ifndef BFGSALGORITHM_H
#define BFGSALGORITHM_H


//Deklaracija funkcija definisanih u BFGSAlgorithm.cpp

struct Variable h_t_matrix (struct Variable A, int i);

struct Variable direct_cinematics_matrix( struct Variable A);

struct Quaternion rotm_to_quats (struct Variable A);

struct Quaternion orientation_error (struct Variable dp_htm, struct Variable ap_htm);

double criterion (struct Variable dh, struct Variable dpc, struct Variable var);

struct Variable gradient (double( *f)(struct Variable, struct Variable, struct Variable),
                                 struct Variable dh, Variable dpc, Variable var);

struct Variable BFGS ( double( *f)(struct Variable, struct Variable, struct Variable),
                      struct Variable dh, struct Variable dpc, struct Variable start_point);

 #endif
