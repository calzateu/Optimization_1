function [f] = factibilidad_minimizar(cuadro)
%% Verificar factibilidad

    fact = cuadro(1:end-1,end) >= 0;    % Se analiza inv(B)*b

    if fact                             % Si el cuadro es factible
        f = 1;
    else                                % Si el cuadro no es factible (existen elementos que pueden minimizar aún la función objetivo)
        f = 0;
    end

end