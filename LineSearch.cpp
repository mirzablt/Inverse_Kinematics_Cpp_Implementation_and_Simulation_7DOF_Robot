#include "LineSearch.h"

/* Funkcija line implementira lednacinu (2.15), a to je linija po kojoj vrsimo
   jednodimenzionalno pretrazivanje po parametru alpha. Ona je odredjena tackom
   'point' i vektorom pravca 'direction'. */

struct Variable line (struct Variable point, struct Variable direction, double alpha){
    
	struct Variable linea = initialize_vector_column(point.X.size());
    
	for (int i = 0; i < point.X.size(); i++){
		linea.X[i][0] = point.X[i][0] + alpha * direction.X[i][0];
	}
    return linea;
}


/* Izvod funkcije f na pravoj odredjenom tackom 'point' i pravcem 'direction', za parametar t. */

double derivative_on_line (double( *f)(struct Variable, struct Variable, struct Variable),
                           struct Variable dh, struct Variable dpc,
                           struct Variable point, struct Variable direction, double t){
    
	double df = f(dh, dpc, line(point, direction, t + h)) -
                f(dh, dpc, line(point, direction, t - h));
    
	return df / (2.0 * h);
}


/* Drugi izvod funkcije f na pravoj odredjenom tackom 'point' i pravcem 'direction', za parametar t. */

double second_derivative (double( *f)(struct Variable, struct Variable, struct Variable),
                          struct Variable dh, struct Variable dpc,
                          struct Variable point, struct Variable direction, double t){
    
	double ddf = derivative_on_line (f, dh, dpc, point, direction, t+h) -
                 derivative_on_line (f, dh, dpc, point, direction, t-h);
    
	return ddf / (2.0 * h); 
} 

/* Funkcija newtonTwoPoints implementira jednodimenzionalno pretrazivanje odredjeno
   sa (2.24), za odredjivanje optimalnog koraka duz linije odredjene pravcem 'direction'
   i tackom 'point'. Kriterij zaustavljanja (eps) je dostizanje minimalne  promjene
   problemske varijable. Pocetna vrijednost x1k je uzeta proizvoljno. */

struct Variable newton_two_points (double( *f)(struct Variable, struct Variable, struct Variable),
                                   struct Variable dh, struct Variable dpc,
                                   struct Variable point, struct Variable direction){
	int i;
	double xk1;
	double eps;
    
	struct Variable XK = point;
	struct Variable optimal_step = initialize_vector_column(1);
    
	double xk = 0.0; 
	double x1k = 0.2; 
    
	do{
		double dxk = xk-x1k;
		double d_derivative_on_line_k = derivative_on_line( f, dh, dpc, XK, direction, xk) -
                                        derivative_on_line( f, dh, dpc, XK, direction, x1k);
        
		xk1 = x1k + (dxk / (1 - ((derivative_on_line(f, dh, dpc, XK, direction, xk)/
                     derivative_on_line(f, dh, dpc, XK, direction, x1k))*
                     (d_derivative_on_line_k / (dxk * second_derivative( f, dh, dpc, XK, direction, xk))))));
        
		eps = fabs(xk1 - xk);
		x1k = xk;
		xk  = xk1;
        
	}while (eps > EPSILON);
    
	optimal_step.X[0][0] = xk1;
    
	return optimal_step;
}

