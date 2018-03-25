function u = controlLaw(posRi, Mv, Lv)
if(Mv==0)
    u = 0;
else
    u = 5*(Lv/Mv-posRi);
end