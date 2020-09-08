function EquationList = CalcShortDist
EquationList.CSC=@CSC;  
EquationList.CCC=@CCC;  
EquationList.CCCC=@CCCC;  
EquationList.CCSC=@CCSC;  
EquationList.CCSCC=@CCSCC; 
end 

function [isok,path] = CSC(x,y,phi)
    formuarList=FormularList;
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = formuarList.LpSpLp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(15,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpSpLp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(15,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpSpLp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(16,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpSpLp(-x,-y,phi); % timeflp + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(16,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpSpRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(13,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpSpRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(13,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpSpRp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(14,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpSpRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(14,:),-t,-u,-v,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function [isok,path] = CCC(x,y,phi)
    formuarList=FormularList;
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = formuarList.LpRmL(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(1,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmL(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(1,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmL(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(2,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmL(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(2,:),-t,-u,-v,0,0);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [isok,t,u,v] = formuarList.LpRmL(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(1,:),v,u,t,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmL(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(1,:),-v,-u,-t,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmL(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(2,:),v,u,t,0,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmL(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(2,:),-v,-u,-t,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function [isok,path] = CCCC(x,y,phi)
    formuarList=FormularList;
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = formuarList.LpRupLumRm(x,y,phi);
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(3,:),t,u,-u,v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRupLumRm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(3,:),-t,-u,u,-v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRupLumRm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(4,:),t,u,-u,v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRupLumRm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(4,:),-t,-u,u,-v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRumLumRp(x,y,phi);
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(3,:),t,u,u,v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRumLumRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(3,:),-t,-u,-u,-v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRumLumRp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(4,:),t,u,u,v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRumLumRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(4,:),-t,-u,-u,-v,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function [isok,path] = CCSC(x,y,phi)
    formuarList=FormularList;
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = formuarList.LpRmSmLm(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(5,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmLm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(5,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmLm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(6,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmLm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(6,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmRm(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(9,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmRm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(9,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmRm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(10,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmRm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(10,:),-t,pi/2,-u,-v,0);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [isok,t,u,v] = formuarList.LpRmSmLm(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(7,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmLm(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(7,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmLm(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(8,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmLm(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(8,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmRm(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(11,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmRm(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(11,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmRm(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(12,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSmRm(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(12,:),-v,-u,pi/2,-t,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function [isok,path] = CCSCC(x,y,phi)
    formuarList=FormularList;
    Lmin = inf;
    type = repmat([RSPathElem.RS_NOP],[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = formuarList.LpRmSLmRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(17,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSLmRp(x,y,phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(17,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSLmRp(x,y,phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(18,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = formuarList.LpRmSLmRp(x,y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPathElem.Type(18,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end