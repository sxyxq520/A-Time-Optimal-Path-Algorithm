function [ IC ] = Inertia_body_frame( ICB, A )

IC = A*ICB*A.';


end

