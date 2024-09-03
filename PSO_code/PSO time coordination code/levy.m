function Levy_step = levy(dimension,beta)
% nvar : �������ĸ���
% beta = 1.5  ����

% ksai = (gamma(1+beta)*sin(beta/2*pi)/gamma(0.5+0.5*beta)/beta/(2^(0.5*beta-0.5)))^(1/beta);

% ����X
% beta = 3/2;
% ����alpha_u
alpha_u = (gamma(1+beta)*sin(pi*beta/2)/(gamma(((1+beta)/2)*beta*2^((beta-1)/2))))^(1/beta);
% ����alpha_v
alpha_v = 1;

% u ,v������̬�ֲ�
u=normrnd(0,alpha_u^2,[1 dimension]);
v=normrnd(0,alpha_v^2,[1 dimension]);
% levy(lamda)
% s = u/((abs(v))^(1/beta));


Levy_step = 0.01.*u./(abs(v).^(1/beta));

end



