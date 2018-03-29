function posNew = positionUpdate(posOld, u, par)

if(par.diffDrive == true)
    posNew = 0*posOld;
    for ii = 1:par.N
        phi         = posOld(ii,end);
        R           = [cos(phi),    0;
                        sin(phi),   0;
                        0,          1];
        posNew(ii,:)= posOld(ii,:) + (R*u(ii,:).'*par.dt).';
    end
else
    posNew  = posOld+u*par.dt;
end