%filtro de Kalman em tempo discreto em LTV (slides M2->54)
function[x,P] = Kalman_predict_function(x,P,F,Q,Gu)

    %Matrix de covariância
    P = F * P * F' + Q;
    
    %mantem matrix P(k) simétrica
    P = (P+P')/2;
    
    %previsão de estado
    x = F*x + Gu;
end