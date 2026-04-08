function xhat_k1 = suspended_estimator( u, y, xhat_k, Ae_ds, Be_ds )

% coder.extrinsic('evalin')
% Ae_d = evalin('base', 'Ae_d');
% Be_d = evalin('base', 'Be_d');
% T   = evalin('base', 'T'  );

xhat_k1 = Ae_ds*xhat_k + Be_ds*[u; y];

end
