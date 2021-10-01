function [costo] = simplex_minimizar(f,A,b)
% f = función de costo objetivo
% A = contantes de desigualdad
% b = recursos (los recursos deben ser positivos para
% garantizar optimalidad de la base inicial)

m = size(A,1);      % Número de ecuaciones
B = eye(m);         % Base inicial identidad
N = A;              % Matriz variables no básicas
CB = zeros(1,m);    % Costos básicos (son las variables de holgura, se suponen en cero)
CN = f;             % Costos no básicos deben ser entragados en la función a minimizar

num_variables = size(f,2); % Número de variables iniciales
variables_holgura = (num_variables+1:num_variables+1 + m)'


%% construcción cuadro simplex

cuadro = [zeros(m,1)       B              inv(B)*N           inv(B)*b;
            1           zeros(1,m)      CB*(inv(B)*N)-CN     CB*(inv(B)*b)]

cuadro = [variables_holgura cuadro]

if (factibilidad_minimizar(cuadro) == 1)
    if (optimalidad_minimizar(cuadro,m) == 1)
        msgbox('cuadro óptimo')
    else
        while (optimalidad_minimizar(cuadro,m) ~= 1)
            [V1, p1] = max(cuadro(end,1+m+2:end-1));                    % Máximo valor columna pivote. V valor, P posición
            columna = cuadro(1:end-1,end)./cuadro(1:end-1,1+m+1+p1);      % Se dividen los recursos entre la columan pivote
            positivo = columna >= 0;                                    % Solo los valores positivos participan
            a = (positivo).*columna;                                    % Guardar los valores positivos
            a(a==0) = inf;                                              % Si existen valores cero se ponen en infinito
            [V2, p2] = min(a);                                          % Valor de la variable de bloqueo (index fila pivote)
            fila = cuadro(p2,:); % Cambié 1+p2 de acá en adelante                                    % Fila pivote
            Epiv = cuadro(p2,1+m+1+p1);                                 % Elemento pivote
            cuadro(p2, 1) = p1; % Mirar si es -1
            for i=1:1:(m+1)
                if i == (p2)
                    cuadro(i,2:end) = cuadro(i,2:end)/Epiv;
                else
                    cuadro(i,2:end) = cuadro(i,2:end) - (cuadro(i,1+m+1+p1)*(cuadro(p2,2:end)/Epiv));
                end
            end
            cuadro
        end
    end       
else
    msgbox('No se puede hacer nada aún')
end
