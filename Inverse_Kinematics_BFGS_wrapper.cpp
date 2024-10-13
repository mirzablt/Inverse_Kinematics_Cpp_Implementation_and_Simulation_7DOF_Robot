
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include <vector>

#include "MatrixLibrary.cpp"
#include "LineSearch.cpp"
#include "BFGSAlgorithm.cpp"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 4
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void Inverse_Kinematics_BFGS_Outputs_wrapper(const real_T *Pose,
			const real_T *Init,
			real_T *Config,
			real_T *CriterionValue)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* Kod u narednim linijama je implementiran u bloku S-Function Builder u mapi Outputs.
   Sve linije koda koje prethode ovom komentaru su automatski generirane pritiskom 
   tastera Build u bloku S-Function Builder. 

   U mapi Libraries u bloku S-Function Builder su navedene implemetirane biblioteke:
     -BFGSAgorithm  - u kojem je implementiran BFGS optimizacijski algoritam;
     -LineSearch    - u kojem je implementiran algoritam jednodimenzionalnog 
                      pretrazivanja a koji se poziva u pomenutom algoritmu;
     -MatrixLibrary - u kojem su implementirane potrebne operacije vektorima i 
                      matricama kao i standardne biblioteke: math, vector.

  Ulazi bloka su:
    -Pose - zeljena poza robota data u formi matrice homogene tranformacije koja
            odredjuje zeljenu poziciju i orijentaciju vrha manipulatora. Ovu matricu
            generira blok planiranja trajektorije (koji nije obuhvacen ovim radom);
    -Init - inicijalna vrijednost vektora zglobnih varijabli;
  i deklarisani u mapi Data Properties/Input ports bloka S-funkcije.

  Izlazi bloka su:
     -Config         - vrijednost vektora zglobnih varijabli robota koje je rezultat 
                       algoritma inverzne kinematike, a za koju vrh manipulatora zauzima
                       zeljenu poziciju i orijentaciju;
     -CriterionValue - vrujednost funkcije kriterija. */


    using namespace std;

	struct Variable start_point, desired_pose_htm;
    struct Variable A, joint_variables_vector;
    
    
    // Inicijalizacija.
    
	desired_pose_htm = initialize_matrix(4, 4);           
    start_point = initialize_vector_column(7);            
    joint_variables_vector = initialize_vector_column(7); 
    
    
    /* DH parametri robotskog manipulatora KUKA iiwa 14R820  specificirani od 
       strane proizvodjaca. */
    
   	double a1 = 0;
	double a2 = 0;
	double a3 = 0;
	double a4 = 0;
	double a5 = 0;
    double a6 = 0;
	double a7 = 0;
	double alpaha1 =  pi/2.0;
	double alpaha2 = -pi/2.0;
	double alpaha3 = -pi/2.0;
	double alpaha4 =  pi/2.0;
	double alpaha5 =  pi/2.0;
	double alpaha6 = -pi/2.0;
    double alpaha7 =  0.0;
	double d1 = 0.360;
	double d2 = 0;
	double d3 = 0.420;
	double d4 = 0;
	double d5 = 0.400;
	double d6 = 0;
    double d7 = 0.126;
    
       
    // Matrica DH parametara KUKA iiwa 14R820 robota.
    
    A = initialize_matrix(7, 4);
    
	A.X[0][0] = a1;     A.X[0][1] = alpaha1;           A.X[0][2] = d1;  
	A.X[1][0] = a2;     A.X[1][1] = alpaha2;           A.X[1][2] = d2; 
	A.X[2][0] = a3;     A.X[2][1] = alpaha3;           A.X[2][2] = d3;  
	A.X[3][0] = a4;     A.X[3][1] = alpaha4;           A.X[3][2] = d4;  
	A.X[4][0] = a5;     A.X[4][1] = alpaha5;           A.X[4][2] = d5;  
	A.X[5][0] = a6;     A.X[5][1] = alpaha6;           A.X[5][2] = d6;
    A.X[6][0] = a7;     A.X[6][1] = alpaha7;           A.X[6][2] = d7; 
    
    
    /* desired_pose_htm - desired pose homogene tranformation matrix. Te je matrica
       homogene transformacije koja odredjuje zeljenu poziciju vrha manipulatora.
       Parametar "Pose" je deklarisan u mapi "Data Properties/Input Ports" i on 
       je ulaz bloka S-funkcije. "Pose" je vektor-vrsta kojeg je potrebno prevesti
       u 4x4 matricu "desired_pose_htm". */
    
    int i, j;
    int k = 0;
    for (i = 0; i < 4; i++){
        for (j = 0; j < 4; j++){
           desired_pose_htm.X[i][j] = Pose[i + j + k];
        }
        k += 3;
    };
    
    
    /* start point je pocetna iteracija algoritma optimizacije. U svakom koraku 
       simulacije pocetna tacka je jednaka vektoru zglobnih varijabli iz prethodnog
       koraka. Parametar "Init" je deklarisan u mapi "Data Properties/Input Ports" 
       i on je ulaz bloka S-funkcije. */
    
   for (i = 0; i < 7; i++){
        start_point.X[i][0] = Init[i];
    };
    
    
   /* Racunanje vektora zglobnih varijabli joint_variables_vector, za odabranu 
      funkciju kriterija datu sa (2.13), za robotski manipulator cija je kinematika
      odredjena matricom DH parametara A, za zeljenu pozu odredjenu sa matricom 
      homogene transformacije desired_pose_htm i pocetnu aproksimaciju start_point
      koja je jednaka vektoru zglobnih varijabli iz prethodnog koraka simulacije.
      Parametar "Config" je deklarisan u mapi "Data Properties/Output Ports" i on
      je izlaz bloka S-funkcije i predstavlja rezultat racunanja implementiranog 
      algoritma inverzne kinematike. */    
    
    joint_variables_vector = BFGS(criterion, A, desired_pose_htm, start_point);
    for (i = 0; i < 7; i++){
        Config[i] = joint_variables_vector.X[i][0];
    };
    
      
   /* Vrijednost funkcije kriterija za robot  cija je kinematika odredjena  matricom
      DH parametara A, za zeljenu poziciju i orijentaciju odredjenu matricom homogene
      transformacije desired_pose_htm, i za BFGS algoritmom izracunati vektor zglobnih
      varijabli joint_Variables vector. Parametar "CriterionValue" je deklarisan u mapi 
      "Data Properties/Output Ports" i on je jedan od izlaza bloka S-funkcije. */
    
    CriterionValue[0] = criterion(A, desired_pose_htm, joint_variables_vector);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


