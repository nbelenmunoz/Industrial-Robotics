function G = computeGravityVector(q, L, m, g_const)
    d4     = q(4);
    m_total = sum(m);
    G = zeros(4,1);
    G(4) = m_total * g_const;
end
