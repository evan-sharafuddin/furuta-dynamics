function xhat_k1 = suspended_estimator( u, y, xhat_k, Ae_d, Be_d )

xhat_k1 = Ae_d*xhat_k + Be_d*[u; y];

end
