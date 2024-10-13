#include "BFGSAlgorithm.h"


/* Funkcija h_t_matrix racuna  matricu homogene transformacije B, odredjena
   relacijom (1.14), na osnovu matrice DH parametara A. A.data[i][0] su parametri a;
   A.data[i][1] su parametri alfa; A.data[i][2] su d parametri d; A.data[i][3] 
   su theta parammetri (zglobne varijable). */

struct Variable h_t_matrix (struct Variable A, int i){
	struct Variable B = initialize_matrix(4, 4);
    
    B.X[0][0] = cos(A.X[i][3]);                   B.X[0][1] = -sin(A.X[i][3])*cos(A.X[i][1]);    
    B.X[1][0] = sin(A.X[i][3]);                   B.X[1][1] =  cos(A.X[i][3])*cos(A.X[i][1]);    
	B.X[2][0] = 0.0;                              B.X[2][1] =  sin(A.X[i][1]);                                  
	B.X[3][0] = 0.0;                              B.X[3][1] =  0.0;                              
   
    B.X[0][2] =  sin(A.X[i][3])*sin(A.X[i][1]);   B.X[0][3] =  A.X[i][0]*cos(A.X[i][3]);
    B.X[1][2] = -cos(A.X[i][3])*sin(A.X[i][1]);   B.X[1][3] =  A.X[i][0]*sin(A.X[i][3]);
    B.X[2][2] =  cos(A.X[i][1]);                  B.X[2][3] =  A.X[i][2];
    B.X[3][2] =  0.0;                             B.X[3][3] =  1.0;
    
    return B; 
} 


/* Jednacina direktne kinematike definise ovisnost pozicije i orijentacije vrha
   manipulatora od zglobnih varijabli. Funkcija direct_cinenematics_matrix daje
   matricu (4x4) koja odredjuje jednacinu direktne kinematike robota, koja se 
   dobije mnozenjem matrica homogene transf. A prema jednacini (1.10). */

struct Variable direct_cinematics_matrix( struct Variable A){
    
	struct Variable M, N;
	N = h_t_matrix(A, 0);
    
	for (int i = 1; ; ){
		M = multiply(N, h_t_matrix(A, i));
		i++;
		if(i >= A.X.size()) break;
		N = multiply(M, h_t_matrix(A, i));
		i++;
		if(i >= A.X.size()) break;	
	} 
	if(A.X.size()%2 == 0) {
        return M;
    } else {
        return N;
    }
} 


/* Orijentacija vrha manipulatora je odredjena matricom rotacije. Ova orijentacija
   se moze izraziti u terminima kvaterniona. Metoda rotation_matrix_to_quats 
   transformise matricu rotacije (koja je jednaka gornjoj trougaonoj matrici 
   matrice homogene transf. A) u kvaternion Q = [s, v0, v1, v2] prema relaciji
   (2.8) uz tretiranje granicnih slucajeva. */

 struct Quaternion rotm_to_quats (struct Variable A){
     
 	struct Quaternion B;
 	double trace = A.X[0][0]+A.X[1][1]+A.X[2][2];
    
 	if (trace > h) {
        
	    double s = 0.5 / sqrt(trace+1.0);
	    B.s = 0.25 / s;
	    B.v[0] = (A.X[2][1] - A.X[1][2]) * s;
	    B.v[1] = (A.X[0][2] - A.X[2][0]) * s;
	    B.v[2] = (A.X[1][0] - A.X[0][1]) * s;
        
	 } else {
         
	 	if (A.X[0][0] - A.X[1][1] >h && A.X[0][0] - A.X[2][2] > h){
			
	 		double s = 2.0 * sqrt(1.0 + A.X[0][0] - A.X[1][1] - A.X[2][2]);
	 		 B.s = (A.X[2][1] - A.X[1][2]) / s;
		         B.v[0] = 0.25 * s;
		         B.v[1] = (A.X[0][1] + A.X[1][0]) / s;
		         B.v[2] = (A.X[0][2] + A.X[2][0]) / s;
             
		 } else if (A.X[1][1] - A.X[2][2]>h){
             
		 	double s = 2.0 * sqrt(1.0 + A.X[1][1] - A.X[0][0] - A.X[2][2]);
		 	 B.s = (A.X[0][2] - A.X[2][0]) / s;
		 	 B.v[0] = (A.X[0][1] + A.X[1][0]) / s;
		 	 B.v[1] = 0.25 * s;
		 	 B.v[2] = (A.X[1][2] + A.X[2][1]) / s;
             
		 } else {
             
		 	double s = 2.0 * sqrt(1.0 + A.X[2][2] - A.X[0][0] - A.X[1][1]);
		 	 B.s = (A.X[1][0] - A.X[0][1]) / s;
		 	 B.v[0] = (A.X[0][2] + A.X[2][0]) / s;
		 	 B.v[1] = (A.X[1][2] + A.X[2][1]) / s;
		 	 B.v[2] = 0.25 * s;	
		 }
	 }
    return B;	
 }
 
 
/* Greska orijentacije izrazava odstupanje trenutne orijentacije vrha manipulatora
   od zeljene orijentacije. Ovo odstupanje se moze izraziti u obliku matrice (2.7).
   Ova matrica se dobije na osnovu matrica dp_htm i ap_htm i ona se moze prevesti
   u kvaternion, ciji vektorski dio (relacija 2.12) opisuje  ovu gresku orijentacije
   u obliku (2.12). Naredna funkcja daje vektorski dio kvaterniona na osnovu
   matrica dp_htm i ap_htm. */
 
  struct Quaternion orientation_error (struct Variable dp_htm, struct Variable ap_htm){
 	
    struct Quaternion D = rotm_to_quats(dp_htm);   /*dp_htm - desired pose homogene transform matrix */
 	struct Quaternion E = rotm_to_quats(ap_htm);   /*ap_htm - actual  pose homogene transform matrix */
 	struct Quaternion delta;
    
 	delta.s = 1.0;
 	delta.v[0] = E.s * D.v[0] - D.s * E.v[0] - (-D.v[2] * E.v[1] + D.v[1] * E.v[2]);
 	delta.v[1] = E.s * D.v[1] - D.s * E.v[1] - ( D.v[2] * E.v[0] - D.v[0] * E.v[2]);
 	delta.v[2] = E.s * D.v[2] - D.s * E.v[2] - (-D.v[1] * E.v[0] + D.v[0] * E.v[1]);
    
 	return delta;	
 }    
  
  
/* U funkciji criterion je implementirana fukcija cilja (2.13) koja izrazava 
   mjeru odstupanja trenutne pozicije i orijentacije vrha manipulatora od njegove
   zeljene pozicije i orijentacije. Funkcija cilja u sebe ukljucuje:
      -positionPart - ovaj dio izrazava rastojanje trenutne pozicije vrha manipulatora
       (koji je funkcija zglobnih varijabli) od zeljene pozicije;
      -orientationPart -  ovaj dio izrazava ostupanje trenutne orijentacije vrh
       a manipulatora (koji je funkcija zglobnih varijabli) od zeljene orijentacije.
       Ovo odstupanje se moze izraziti vektorskim dijelom kvaterniona.
   Matrica tezinskih koeficijenata pozicije Mp iz (2.13) je dijagonalna i njeni
   elementi na glavnoj dijagonali su uzeti da su jednaki i iznose: positionWeights. 
   Matrica tezinskih koeficijenata orijentacije Mo iz (2.13) je dijagonalna 
   i njeni elementi na glavnoj dijagonali su uzeti da su jednaki  i iznose: 
   orientationWeights. 
   Zeljena pozicija i orijentacija vrha manipulatora se dobiju na osnovu matrice 
   homogene transf. dp_htm. Trenutna poz. i orij. se dobiju iz jednacine direktne 
   kinematike, koja se dobije na osnovu dh parametara robota (koje cine i zglobne
   varijable qi). U funkciji cilja 'criterion', var.data[i][0] (i=1,2,...,n)
   su varijable one predstavljaju zglobne promjenjive q1, q2, ...,qn.   */
  
 double criterion (struct Variable dh, struct Variable dp_htm, struct Variable var){
     
	for (int i = 0; i < dh.X.size(); i++){
		dh.X[i][3] = var.X[i][0];
	}
	struct Variable dcm = direct_cinematics_matrix(dh);
	struct Quaternion delta = orientation_error (dp_htm, dcm);
   	double position_part = 0.0;
   	double orientation_part = 0.0;
    
    const double position_weights = 1;
    const double orientation_weights = 0.01;
    
   for(int i = 0; i < 3; i++){
       position_part += position_weights * pow(dp_htm.X[i][3] - dcm.X[i][3], 2);
       orientation_part += orientation_weights * pow(delta.v[i], 2);
   };
   
	return position_part + orientation_part;
}

 
/* Metod grad racuna gradijent funkcije f po promjenjivoj 'var'. Kako racunamo 
   gradijent funkcije cilja, to ovom metodu moramo proslijediti i parametre 
   analogno kao u slucaju metoda u kojem je implementirana funk. cilja.  */
 
struct Variable gradient (double(*f)(struct Variable, struct Variable, struct Variable),
                          struct Variable dh, struct Variable dpc, struct Variable var){

    struct Variable var_1, var_2;
    struct Variable partial_derivatives;
    partial_derivatives = initialize_vector_column(var.X.size());
    
	for(int i = 0; i < var.X.size(); i++){
		         var_1 = var;
	             var_2 = var;
		var_1.X[i][0] += h;
	    var_2.X[i][0] -= h;
		partial_derivatives.X[i][0] = ( f(dh, dpc, var_1) - f(dh, dpc, var_2) ) / (2.0 * h);
	}
	return partial_derivatives;	
}


/* Implementacija algoritma opisanog u 2.5. Algoritam odredjuje vektor zglobnih 
   varijabli za kojeg funkcija cilja f ima minimalnu vrijednost. Ta vrijednost 
   je bliska nuli (EPSILON = 0.0001). U idealnom slucaju ta vrijednost je nula,
   sto odgovara slucaju da se aktuelna pozicija (i orijentacija) i zeljena poz.
   (i orij.) vrha manipulatora poklapaju. 
   Zeljena pozicija i orijentacija vrha manipulatora se dobiju na osnovu matrice 
   homogene transf. dp_htm, koju generira algoritam planiranja trajektorije. Ova
   matrica je ulaz bloka S-funkcije. Trenutna poz. i orij. se dobiju iz jednacine
   direktne kinematike, koja se dobije na osnovu dh parametara robota koje cine 
   zglobne varijable qi.
   Xk.data[i] (i=1,2,...,n) je vektor problemskih varijabli (i predstavlja vektor 
   zglobnih varijabli q1, q2, ...,qn). */

struct Variable BFGS ( double( *f)(struct Variable, struct Variable, struct Variable),
                       struct Variable dh, struct Variable dp_htm, struct Variable start_point){
    
	struct Variable Xk, Xk1, DXk;
	       Variable Gk, Gk1, DGk;
	       Variable step, Rk;
           
	       Variable Hk, Hk1;
	       Variable Mk, Nk;
           
	double gamma, beta;	
		Xk = start_point;
        Hk = eye(start_point.X.size());
    
	while (vector_norm(gradient(f, dh, dp_htm, Xk)) > EPSILON){        // Kriterij zaustavljanja dat sa (2.59).
        
		Gk    = gradient(f, dh, dp_htm, Xk);
		Rk    = c(multiply(Hk, Gk), -1);                               // Pravac pretrazivanja dat sa (2.60). */
        
		step  = newton_two_points (f, dh, dp_htm, Xk, Rk);             // Jednodimenzionalno pretrazivanje (2.61). */
        
		Xk1   = add(Xk, multiply(Rk, step));                           // Racunanje nove aproksimacije (2.62). */
		Gk1   = gradient(f, dh, dp_htm, Xk1);
        
		DXk   = subtract(Xk1, Xk);                                     // Razlika data sa(2.63a).
		DGk   = subtract(Gk1, Gk);                                     // Razlika data sa(2.63b).
        
		gamma = (multiply( transpose(DXk), DGk)).X[0][0];
		beta  = (multiply( multiply( transpose(DGk), Hk), DGk)).X[0][0];
        
		Mk    = c(multiply(DXk, transpose(DXk)), (gamma + beta) / pow(gamma,2));
		Nk    = c( add( multiply( multiply(Hk, DGk), transpose(DXk)),
                   multiply( multiply(DXk, transpose(DGk)), Hk)), -1.0/gamma);
        
		Hk1   = add( add(Hk, Mk), Nk);                                 // Racunanje aproksimacije inverznog Hessiana H(k+1) prema (2.56).
        
		Hk    = Hk1;
		Xk    = Xk1;	
	}
    return Xk1;
}
