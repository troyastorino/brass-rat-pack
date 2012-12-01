m1 = 1
m2 = 1
m3 = 1
mr = 1
l1 = 1
l2 = 1
l3 = 1
L = 1
lc1 = 1
lc2 = 1
lc3 = 1
I1 = 1
I2 = 1
I3 = 1

def total_energy(q, q_dot):

  T = 0.5*((L^2)*mc*q_dot[4]^2 + I1*q_dot[5]^2 + 
   I2*(q_dot[5] + q_dot[1])^2 + 
   I3*(q_dot[5] + q_dot[1] + q_dot[2])^2 + 
   m1*((L^2)*q_dot[4][t]^2 + 
       2*L*lc1*cos(q[4] - q[5])*q_dot[4]*q_dot[5] + (lc1^2)*q_dot[5]^2)+ 
   m2*((L*cos(q[4])*q_dot[4] + 
         l1*cos(q[5])*q_dot[5] + 
         lc2*cos(q[5] + q[1])*(q_dot[5] + q_dot[1]))^2 + (L*sin(q[4])*q_dot[4] + l1*sin(q[5])*q_dot[5] + 
         lc2*sin(q[5] + q[1])*(q_dot[5] + q_dot[1]))^2) + 
   m3*((L*cos(q[4])*q_dot[4] + 
         l1*cos(q[5])*q_dot[5] + 
         l2*cos(q[5] + q[1])*(q_dot[5] + q_dot[1]) + 
         lc3*cos(q[5] + q[1] + q[2])*(q_dot[5] + q_dot[1] + q_dot[2]))^2 + (L*sin(q[4])*q_dot[4] + l1*sin(q[5])*q_dot[5] + 
         l2*sin(q[5] + q[1])*(q_dot[5] + q_dot[1]) + 
         lc3*sin(q[5] + q[1] + q[2])*(q_dot[5] + q_dot[1] + q_dot[2]))^2))
    
  V = -g*(L*(m1 + m2 + m3 + mr)*cos(q[4]) + (lc1*m1 + l1*(m2 + m3))*cos(q[5]) + (lc2*m2 + l2*m3)*cos(q[5] + q[1]) + 
   lc3*m3*cos(q[5] + q[1] + q[2]))

  return T+V
