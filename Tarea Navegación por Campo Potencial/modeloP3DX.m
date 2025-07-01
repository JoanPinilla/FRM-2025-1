function dx = modeloP3DX(t, x, u)
    % Modelo cinemático del Pioneer P3DX
    % x = [x; y; theta]
    % u = [v; w]
    v = u(1);   % velocidad lineal
    w = u(2);   % velocidad angular
    dx = zeros(3,1);
    dx(1) = v * cos(x(3));
    dx(2) = v * sin(x(3));
    dx(3) = w;
end
