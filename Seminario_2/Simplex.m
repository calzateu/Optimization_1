% Método Simplex

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c = [-1 -2 -3];   % Vector de coeficientes

A = [1 1 1     % Matriz de restricciones
     0 1 1 
     0 0 1]; 

b = [1       % Vector de recursos
     2
     3];
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

maxIteraciones = 100;  % Número máximo de iteraciones

r=size(b,1); % Número de restricciones

num_variables = size(c,2);
%variables_holgura = zeros(r+1,1);
variables_holgura = (num_variables+1:2*r+1)';

tablaSimplex = [A eye(r) b     % Tablero Simplex con los datos ordenados
              c zeros(1,r) 0];

tablaSimplex = [variables_holgura tablaSimplex];
          
for i = 1:maxIteraciones 
    tablaSimplex
    salida = tablaSimplex(end, 1:end-1)<0;
    if(salida==0)
        break;
    end
    
    [m cp] = min(tablaSimplex(end,:)); % Encontramos la columna pivote hallando el minimo
    
    ratio = tablaSimplex(:,end)./tablaSimplex(:,cp); % Dividimos cada elemento del vector de recursos entre cada elemento de la columna pivote
    
    indicador = ratio <= 0; % Encontramos la posición de los elementos de ratio que sean menores o iguales que cero
    
    ratio(indicador) = inf; % Volvemos infinito los elementos de ratio que sean menores o iguales que cero
    
    [rm fp] = min(ratio); % Encontramos la fila pivote hallando el minimo positivo de ratio
    
    tablaSimplex(fp, 1) = cp-1;
    
    tablaSimplex(fp,:) = tablaSimplex(fp,:)/tablaSimplex(fp,cp); % Dividimos la fila pivote entre el pivote que es la intersección
    
    for j = 1 : size(tablaSimplex,1)
        if(j==fp)
            continue;
        end
        tablaSimplex(j,2:end) = tablaSimplex(j,2:end) - (tablaSimplex(j,cp)*tablaSimplex(fp,2:end));
    end
end