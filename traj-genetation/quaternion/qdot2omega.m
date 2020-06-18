function omega=qdot2omega(q,qdot)
    omega = dqMat(q)*qdot(:);
end