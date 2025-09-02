: Altered version of channel block model described in Glasgow et al. (2017) JNeuro paper.
: Model originally based on Erreger, Dravid, Banke, Wyllie, & Traynelis (2005). 
: Built from 2OpStCaNMDAR.mod
: 2 Symmetric "Arms" - Mem-bound and unbound. 
: 2 "Planes" - Ca2+-bound and unbound. Transition b/t planes is dependent on intracellular [Ca2+]. Ca2+-bound plane possesses additional
: desensitized state, otherwise rates are identical b/t planes (prevents violation of microscopic reversibility).
: 
: 
:
:                                        SUMMARY OF MODEL AND RATE CONSTANTS
:
:
:                                     A2Rd                                      A2RdM
:                                   /   |                                        |    \
:                                  /    |                                        |     \
:                                 /     |                                    kd1M|      \
:                                |   kd1|kd0                                     |kd0M   |
:                 +2A          +A       |                  +M                    |       |
:                 2*ka1        ka1      |     kg1          k1M           kg0M    |    2*ka0M        ka0M
:             R <------> AR <--------> A2R <------> A2Ro <---_--> A2RoM <----> A2RM <--------> ARM <------> RM
:             |   ka0    |   2*ka0      |     kg0    |     k0M     |     kg1M    |     ka1M     |   2*ka1M  |
:             |          |       |      |            |             |             |     +A       |    +2A    |
:             |kCa0      |kCa0   |      |kCa0        |kCa0         |kCa0         |kCa0   |      |kCa0       |kCa0
:         kCa1|      kCa1|   kCa1|kCa0  |        kCa1|         kCa1|             |       |  kCa1|       kCa1|
:             |          |       |      |            |             |         kCa1|       |      |           |
:             |          |       |  kCa1|            |             |             |   kCa1|kCa0  |           |
:             |  +2A     |     +A       |            |     +M      |             |       |      |           |
:             |  2*ka1   |     ka1      |     kg1    |     k1M     |     kg0M    |     2*ka0M   |    ka0M   |
:            cR <-----> cAR <--------> cA2R <-----> cA2Ro <----> cA2RoM <----> cA2RM <-------> cARM <----> cRM
:                ka0         2*ka0      / \   kg0          k0M           kg1M   / \     ka1M         2*ka1M
:                                |     /   \                                   /   \    +A           +2A
:                                | kd1/     \                           kcdd0M/     \    |
:                                |   /kd0    \kcdd0                          /kcdd1M \kd1M
:                                |  /    kcdd1\                             /     kd0M\  |
:                                | /           \                           /           \ |
:                              cA2Rd         cA2Rcdd                    cA2RcddM        cA2RdM  
:
:       
NEURON {
    POINT_PROCESS CaPlaneNMDARwMem
	RANGE A, M, Ca, Vm, nrecepts, scg, Vrev, Popen
	RANGE ka1, ka0, kd1, kd0, kg0, kg1, kcdd1, kcdd0
    RANGE k1M, k0M, ka1M, ka0M, kd1M, kd0M, kg0M, kg1M, kcdd1M, kcdd0M
    RANGE R, AR, A2R, A2Rd, A2Ro 
    RANGE RM, ARM, A2RM, A2RdM, A2RoM
    RANGE kCa1, kCa0, cR, cAR, cA2R, cA2dR, A2Rcdd, cA2Ro 
    RANGE cRM, cARM, cA2RM, cA2RdM, A2RcddM, cA2RoM
	NONSPECIFIC_CURRENT Inmda
	THREADSAFE
}

UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
	(uS) = (microsiemens)
    (pS) = (picosiemens)
	(umho) = (micromho)
	(mM) = (milli/liter)
	(uM) = (micro/liter)
}


PARAMETER
{    
:  CURRENT CONTROL
    nrecepts = 300                    :number of NMDA receptors
    scg = 50             (pS)         :single channel conductance 
    Vrev = 0             (mV)         :reversal potential
     
:RATES
: Transition b/t planes
    kCa1 = 1e-3         (uM-1 s-1)    :Ca "binding"
    kCa0 = 0.75e-3      (ms-1)        :Ca "unbinding"

:Unblocked Arm
    ka1 = 31.6           (uM-1 s-1)   :Agonist binding
    ka0 = 1010e-3        (ms-1)       :Agonist unbinding
:  Gating
    kg1 = 4772.41e-3     (ms-1)       :opening
    kg0 = 557e-3         (ms-1)       :closing  
 
:  Desensitization
    kcdd1 = 32.12e-3     (ms-1)       :into Ca-dependent desen
    kcdd0 = 2.92e-3      (ms-1)       :out of CDD
    kd1 = 3e-3        (ms-1)       :into desens
    kd0 = 3e-3        (ms-1)       :out of desens

:Blocked Arm
    k1M = 1e-3           (uM-1 s-1)  :Block rate
    k0M = 1e-3           (ms-1)      :Unblock rate
    ka1M = 31.6           (uM-1 s-1)  :Agonist binding
    ka0M = 1010e-3        (ms-1)      :Agonist unbinding
:  Gating                                                
    kg1M = 4772.41e-3     (ms-1)      :opening
    kg0M = 53.017e-3      (ms-1)      :closing  
:  Desensitization
    kcdd1M = 64.24e-3     (ms-1)      :into Ca-dependent desen
    kcdd0M = 0.6e-3       (ms-1)      :out of CDD
    kd1M = 0.54e-3        (ms-1)      :into desens
    kd0M = 0.0125e-3      (ms-1)      :out of desens
  
}


ASSIGNED 
{
  A       (uM)        :FROM 0 TO 1000
  M       (uM)        :FROM 0 TO 100
  Inmda   (pA)        :FROM -1000 TO 10 (pA)
  Vm      (mV)        :FROM -70 TO 40 (mV)
  Ca      (uM)        :FROM 0 TO 100
    
}

STATE

{: Value of each state = fraction of all receptors in state

:Unbound Plane   
    :Unblocked arm
      R       
      AR     
      A2R  
      A2Rd      
      A2Ro   
    :Blocked arm
      RM      
      ARM
      A2RM
      A2RdM    
      A2RoM   
:Ca2+-bound plane
     :Unblocked arm
      cR       
      cAR     
      cA2R  
      cA2Rd
      cA2Rcdd
      cA2Ro   
    :Blocked arm
      cRM      
      cARM 
      cA2RM
      cA2RdM
      cA2RcddM
      cA2RoM   
:General
      Popen
}

INITIAL
{
  R = 1
}

BREAKPOINT
{
    SOLVE kstates METHOD sparse
   
    Vm = -65
    
    Popen = A2Ro + cA2Ro
    Inmda = nrecepts * Popen * scg * ((Vm - Vrev)/1000)
}

KINETIC kstates {
:Unbound Plane
   :Unblocked, agonist binding
       ~ A + R <-> AR           (2*ka1, ka0)
       ~ A + AR <-> A2R         (ka1, 2*ka0)
   :Unblocked, gating
       ~ A2R <-> A2Ro           (kg1, kg0)
   :Unblocked, desensitization
       ~ A2R <-> A2Rd           (kd1, kd0)
   :Blocker binding
       ~ A2Ro + M <-> A2RoM     (k1M, k0M)
   :Blocked, agonist binding
       ~ A + RM <-> ARM         (2*ka1M, ka0M)
       ~ A + ARM <-> A2RM       (ka1M, 2*ka0M)
   :Blocked, gating
       ~ A2RM <-> A2RoM         (kg1M, kg0M)
   :Blocked, desensitization
       ~ A2RM <-> A2RdM         (kd1M, kd0M)
:Ca2+-bound Plane
   :Unblocked, agonist binding
       ~ A + cR <-> cAR         (2*ka1, ka0)
       ~ A + cAR <-> cA2R       (ka1, 2*ka0)
   :Unblocked, gating
       ~ cA2R <-> cA2Ro         (kg1, kg0)
   :Unblocked, desensitization
       ~ cA2R <-> cA2Rd         (kd1, kd0)
       ~ cA2R <-> cA2Rcdd        (kcdd1, kcdd0)
   :Blocker binding
       ~ cA2Ro + M <-> cA2RoM   (k1M, k0M)
   :Blocked, agonist binding
       ~ A + cRM <-> cARM       (2*ka1M, ka0M)
       ~ A + cARM <-> cA2RM     (ka1M, 2*ka0M)
   :Blocked, gating
       ~ cA2RM <-> cA2RoM       (kg1M, kg0M)
   :Blocked, desensitization
       ~ cA2RM <-> cA2RdM       (kd1M, kd0M)
       ~ cA2RM <-> cA2RcddM      (kcdd1M, kcdd0M)
:Transitions b/t Planes
       ~ R <-> cR              (Ca*kCa1, kCa0)
       ~ AR <-> cAR            (Ca*kCa1, kCa0)
       ~ A2R <-> cA2R          (Ca*kCa1, kCa0)
       ~ A2Rd <-> cA2Rd        (Ca*kCa1, kCa0)
       ~ A2Ro <-> cA2Ro        (Ca*kCa1, kCa0)
       ~ RM <-> cRM            (Ca*kCa1, kCa0)
       ~ ARM <-> cARM          (Ca*kCa1, kCa0)
       ~ A2RM <-> cA2RM        (Ca*kCa1, kCa0)
       ~ A2RdM <-> cA2RdM      (Ca*kCa1, kCa0)
       ~ A2RoM <-> cA2RoM      (Ca*kCa1, kCa0)
          
CONSERVE R + AR + A2R + A2Rd + A2Ro + RM + ARM + A2RM + A2RdM + A2RoM + cR + cAR + cA2R + cA2Rd + cA2Rcdd + cA2Ro + cRM + cARM + cA2RM + cA2RdM + cA2RcddM + cA2RoM = 1
}  

