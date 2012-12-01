m1 = 0.308
m2 = 0.337
m3 = 0.163
mr = 0

l2 = 0.120
L = 2
lc2 = 0.057
lc3 = 0.0969


I2 = 1.01e-3
I3 = 2*4580.56e-7

def total_energy(q, q_dot):

  theta_3 = q[2]
  l1 = sqrt(0.065^2+(0.220*cos(theta_3))^2)
  lc1 = sqrt(0.027^2+(0.056*cos(theta_3))^2)
  b1 = 1.444e-3 + (0.056*cos(theta_3))^2
  b2 = (0.0396*cos(theta_3))^2+0.027^2
  I1 = 0.126*b1+0.182*b2+0.0055*(cos(theta_3))^2

  T = 0.5*((L^2)*mc*q_dot[3]^2 + I1*q_dot[4]^2 +
           
   I2*(q_dot[4] + q_dot[0])^2 +
           
   I3*(q_dot[4] + q_dot[0] + q_dot[1])^2 +
           
   m1*((L^2)*q_dot[3]^2 +
       
       2*L*lc1*cos(q[3] - q[4])*q_dot[3]*q_dot[4] + (lc1^2)*q_dot[4]^2)+
           
   m2*((L*cos(q[3])*q_dot[3] +
        
         l1*cos(q[4])*q_dot[4] +
        
         lc2*cos(q[4] + q[0])*(q_dot[4] + q_dot[0]))^2 + (L*sin(q[3])*q_dot[3] + l1*sin(q[4])*q_dot[4] +
                                                          
         lc2*sin(q[4] + q[0])*(q_dot[4] + q_dot[0]))^2) +
           
   m3*((L*cos(q[3])*q_dot[3] +
        
         l1*cos(q[4])*q_dot[4] +
        
         l2*cos(q[4] + q[0])*(q_dot[4] + q_dot[0]) +
        
         lc3*cos(q[4] + q[0] + q[1])*(q_dot[4] + q_dot[0] + q_dot[1]))^2 + (L*sin(q[3])*q_dot[3] + l1*sin(q[4])*q_dot[4] +
                                                                            
         l2*sin(q[4] + q[0])*(q_dot[4] + q_dot[0]) +
                                                                            
         lc3*sin(q[4] + q[0] + q[1])*(q_dot[4] + q_dot[0] + q_dot[1]))^2))
    
  V = -g*(L*(m1 + m2 + m3 + mr)*cos(q[3]) + (lc1*m1 + l1*(m2 + m3))*cos(q[4]) + (lc2*m2 + l2*m3)*cos(q[4] + q[0]) +
          
   lc3*m3*cos(q[4] + q[0] + q[1]))

  return T+V
