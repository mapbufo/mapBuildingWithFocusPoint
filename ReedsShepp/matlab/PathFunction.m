
function path = FindRSPath(x,y,phi)
    calcShortDist=CalcShortDist;
    rmin = 5; %minimum turning radius
    x = x/rmin;
    y = y/rmin;
    [isok1,path1] = calcShortDist.CSC(x,y,phi);
    [isok2,path2] = calcShortDist.CCC(x,y,phi);
    [isok3,path3] = calcShortDist.CCCC(x,y,phi);
    [isok4,path4] = calcShortDist.CCSC(x,y,phi);
    [isok5,path5] = calcShortDist.CCSCC(x,y,phi);
    isoks = [isok1, isok2, isok3, isok4, isok5];
    paths = {path1, path2, path3, path4, path5};
    Lmin = inf;
    for i = 1:5
        if isoks(i) == true
            elem = paths{i};
            if Lmin > elem.totalLength
                Lmin = elem.totalLength;
                path = elem;
            end
        end
    end
end

function PlotPath(path)
    type = path.type;
%     x = [];
%     y = [];
    seg = [path.t,path.u,path.v,path.w,path.x];
    pvec = [0,0,0];
    rmin = 5;
    for i = 1:5 
        if type(i) == RSPathElem.RS_STRAIGHT
            theta = pvec(3);
            dl = rmin*seg(i);
            dvec = [dl*cos(theta), dl*sin(theta), 0];
            dx = pvec(1)+linspace(0,dvec(1));
            dy = pvec(2)+linspace(0,dvec(2));
 
            pvec = pvec+dvec;
        elseif type(i) == RSPathElem.RS_LEFT
            theta = pvec(3);
            dtheta = seg(i);
            cenx = pvec(1)-rmin*sin(theta);
            ceny = pvec(2)+rmin*cos(theta);
            t = theta-pi/2+linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
 
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl = dtheta;
        elseif type(i) == RSPathElem.RS_RIGHT
            theta = pvec(3);
            dtheta = -seg(i);
            cenx = pvec(1)+rmin*sin(theta);
            ceny = pvec(2)-rmin*cos(theta);
            t = theta+pi/2+linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
 
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl = -dtheta;
        else
            % do nothing
        end
        if dl > 0
            plot(dx,dy,'b');
        else
            plot(dx,dy,'r');
        end
        hold on
    end
    hold off
    axis equal
end