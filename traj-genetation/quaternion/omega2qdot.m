function qdot=omega2qdot(q,omega)
    qdot = omegaQ(q)'*[0;omega(:)]/2;
end