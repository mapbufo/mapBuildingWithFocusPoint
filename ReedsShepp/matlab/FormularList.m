function EquationList = FormularList
EquationList.LpSpLp=@LpSpLp; % 8.1, see paper
EquationList.LpSpRp=@LpSpRp; % 8.2
EquationList.LpRmL=@LpRmL; % 8.3 / 8.4
EquationList.LpRupLumRm=@LpRupLumRm; % 8.7
EquationList.LpRumLumRp=@LpRumLumRp; % 8.8
EquationList.LpRmSmLm=@LpRmSmLm; % 8.9
EquationList.LpRmSmRm=@LpRmSmRm; % 8.10
EquationList.LpRmSLmRp=@LpRmSLmRp; % 8.11
end 

function v = mod2pi(x)
    v = rem(x,2*pi);
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end

function [tau,omega] = tauOmega(u,v,xi,eta,phi)
    delta = mod2pi(u-v);
    A = sin(u)-sin(delta);
    B = cos(u)-cos(delta)-1;
    t1 = atan2(eta*A-xi*B,xi*A+eta*B);
    t2 = 2*(cos(delta)-cos(v)-cos(u))+3;
    if t2 < 0
        tau = mod2pi(t1+pi);
    else
        tau = mod2pi(t1);
    end
    omega = mod2pi(tau-u+v-phi);
end
% formula 8.1
function [isok,t,u,v] = LpSpLp(x,y,phi)
    [t,u] = cart2pol(x-sin(phi),y-1+cos(phi));
    if t >= 0
        v = mod2pi(phi-t);
        if v >= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end
% formula 8.2
function [isok,t,u,v] = LpSpRp(x,y,phi)
    [t1,u1] = cart2pol(x+sin(phi),y-1-cos(phi));
    if u1^2 >= 4
        u = sqrt(u1^2-4);
        theta = atan2(2,u);
        t = mod2pi(t1+theta);
        v = mod2pi(t-phi);
        if t >= 0 && v >= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.3/8.4
function [isok,t,u,v] = LpRmL(x,y,phi)
    xi = x-sin(phi);
    eta = y-1+cos(phi);
    [theta,u1] = cart2pol(xi,eta);
    if u1 <= 4
        u = -2*asin(u1/4);
        t = mod2pi(theta+u/2+pi);
        v = mod2pi(phi-t+u);
        if t >= 0 && u <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.7
function [isok,t,u,v] = LpRupLumRm(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    rho = (2+sqrt(xi^2+eta^2))/4;
    if rho <= 1
        u = acos(rho);
        [t,v] = tauOmega(u,-u,xi,eta,phi);
        if t >= 0 && v <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.8
function [isok,t,u,v] = LpRumLumRp(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    rho = (20-xi^2-eta^2)/16;
    if rho >= 0 && rho <= 1
        u = -acos(rho);
        if u >= -pi/2
            [t,v] = tauOmega(u,u,xi,eta,phi);
            if t >=0 && v >=0
                isok = true;
                return
            end
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.9
function [isok,t,u,v] = LpRmSmLm(x,y,phi)
    xi = x-sin(phi);
    eta = y-1+cos(phi);
    [theta,rho] = cart2pol(xi,eta);
    if rho >= 2
        r = sqrt(rho^2-4);
        u = 2-r;
        t = mod2pi(theta+atan2(r,-2));
        v = mod2pi(phi-pi/2-t);
        if t >= 0 && u <= 0 && v <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.10
function [isok,t,u,v] = LpRmSmRm(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    [theta,rho] = cart2pol(-eta,xi);
    if rho >= 2
        t = theta;
        u = 2-rho;
        v = mod2pi(t+pi/2-phi);
        if t >= 0 && u <= 0 && v <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.11
function [isok,t,u,v] = LpRmSLmRp(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    [~,rho] = cart2pol(xi,eta);
    if rho >= 2
        u = 4-sqrt(rho^2-4);
        if u <= 0
            t = mod2pi(atan2((4-u)*xi-2*eta,-2*xi+(u-4)*eta));
            v = mod2pi(t-phi);
            if t >= 0 && v >= 0
                isok = true;
                return
            end
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end
