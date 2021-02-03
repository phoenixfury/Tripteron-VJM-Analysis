function [Jt] = Jt(d,q,theta,Tbase,Ttool,leg_no)
q1 = q(1);
q2 = q(2);
q3 = q(3);
L1 = 1;
L2 = 1;
 
T = FK_VJM(d,q,theta,Tbase,Ttool,leg_no);
 T(1:3,4) = [0;0;0];
 
if leg_no ==1
    Tplat = Tx(-0.1*sin(pi/6))*Ty(0.1*cos(pi/6));
elseif leg_no ==2
    Tplat = Tx(0.1) ;
else
    Tplat = Tx(-0.1*sin(pi/6))*Ty(-0.1*cos(pi/6));
end

 Td = Tbase * Tz(d) *Tzd(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
        
  J1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
  Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Txd(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Tyd(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
  Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tzd(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
    Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rxd(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J5 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
      Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ryd(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J6 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
      Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rzd(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J7 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
      Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Txd(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J8 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
      Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Tyd(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J9 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
      Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tzd(theta(10))*Rx(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J10 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
      Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rxd(theta(11))*Ry(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J11 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
      Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ryd(theta(12))*Rz(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J12 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
      Td = Tbase * Tz(d) *Tz(theta(1))...
          * Rz(q1) * Tx(L1)...
          *Tx(theta(2))*Ty(theta(3))*Tz(theta(4))*Rx(theta(5))*Ry(theta(6))*Rz(theta(7))...
          *Rz(q2) * Tx(L2)...
          *Tx(theta(8))*Ty(theta(9))*Tz(theta(10))*Rx(theta(11))*Ry(theta(12))*Rzd(theta(13))...
            *Rz(q3)*Ttool*Tplat/T ;
  J13 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]';
  
  Jt = [J1 J2 J3 J4 J5 J6 J7 J8 J9 J10 J11 J12 J13];
end

