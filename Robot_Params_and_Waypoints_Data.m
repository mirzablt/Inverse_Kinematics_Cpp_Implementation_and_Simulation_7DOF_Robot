
% Importovanje parametara KUKA iiwa 14 R820 robotskog manipulatora
% iz .urdf filea
robotArm = importrobot('iiwa14.urdf');
robotArm.DataFormat = 'column';

% Inicijalne vrijednosti 
numJoints = numel(robotArm.homeConfiguration);
initConfig = robotArm.homeConfiguration;
initVelocity = zeros(numJoints,1);
ikInitGuess = robotArm.homeConfiguration;
maxWaypoints = 7;

% Perioda generisanja uzoraka trajektorije (s)
Ts = 0.1;

%Pocetna putna tacka trajektorije
homeConfigHomTr = getTransform(robotArm,robotArm.homeConfiguration,'world','iiwa_link_ee_kuka');

% Putne tacke trajektorije (m)
% Home poza robota je ujedno i njegova singularna konfiguracija,
% pa je na svaki clan vektora pocetne poz. vrha manipulatora dodat
% broj blizak nuli - eps
eps = 0.0001;
waypoints =  [-homeConfigHomTr(1:3,4)' + eps;   %T1
              0.5  -0.5   0.3;                  %T2   
              0.5  -0.5   0.3;                  %T2' 
              0.5  -0.5   0.7;                  %T3
              0.5  -0.5   0.7;                  %T3' 
              0   -0.75   0.7;                  %T4
             -0.5  -0.5   0.7]';                %T5
          
% Orijentacija vrha manipulatora u putnim tackama  (rad)      
orientations = [0     0     0; 
                0     pi/3  0;
                0     0     pi/2; 
                0     0     pi/2;
                0     pi/8  pi/8;
                0     0     pi/4;
                0    -pi/8 -pi/8]';
                
% Vremena u kojima vrh manipulatora pristiže u putne tacke (s)           
waypointTimes = [0 5 6.5 9.5 11 13 15];

% Brzine vrha manipulatora u putnim tackama (m/s)
waypointVels = 0.1 *[0    0    0;
                     0    0    0; 
                     0    0    0; 
                     0    0    0;
                     0    0    0;
                    -3    0    0
                     0    0    0]';
                     
% Ubrzanja vrha manipulatora u putnim tackama (m/s^2)                 
waypointAccels = zeros(size(waypointVels));



  
