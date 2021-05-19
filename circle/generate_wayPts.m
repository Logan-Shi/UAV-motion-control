function wayPts = generate_wayPts(point1,point2,r)
wayPts = zeros(3,8);
vec = point2 - point1;
vec90 = [-vec(2),vec(1)];
wayPts(1:2,1) = point1 - r * vec/norm(vec);
wayPts(1:2,2) = point1 - r * vec90/norm(vec90);
wayPts(1:2,3) = (point1 + point2)/2;
wayPts(1:2,4) = point2 + r * vec90/norm(vec90);
wayPts(1:2,5) = point2 + r * vec/norm(vec);
wayPts(1:2,6) = point2 - r * vec90/norm(vec90);
wayPts(1:2,7) = wayPts(1:2,3);
wayPts(1:2,8) = point1 + r * vec90/norm(vec90);
wayPts(1:2,9) = wayPts(1:2,1);
end