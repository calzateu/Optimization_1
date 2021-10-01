function [d] = optimalidad_maximizar(cuadro,m)
%% Verificar optimalidad
    
    % Aquí hay que cambia
    Opt = cuadro(end, 1+m+2:end-1) >= 0;    % Si todos los componentes son menores o iguales a cero el cuadro es optimo
    
    if Opt                                  % La base es óptima
        d = 1;
    else                                    % Si la base no es óptima (existen elementos que pueden minimizar aún la función objetivo)
        d = 0;
    end
    
end