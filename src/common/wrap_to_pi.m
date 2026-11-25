function a = wrap_to_pi(a)
%WRAP_TO_PI Wrap angles to [-pi, pi). Base-MATLAB replacement for wrapToPi.
a = mod(a + pi, 2*pi) - pi;
end
