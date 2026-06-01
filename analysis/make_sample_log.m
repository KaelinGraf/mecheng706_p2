function make_sample_log(outfile)
% MAKE_SAMPLE_LOG  Write a SYNTHETIC sweep log in the firmware's output format.
%
%   make_sample_log()            writes ./sample_sweep_log.txt
%   make_sample_log(outfile)     writes the given path
%
% This is fake data for exercising plot_sweep.m before any hardware run - it is
% NOT measured. The numbers come from a toy model (Gaussian angular response,
% inverse-square range falloff, the outer cells peak-shifted +/-5 deg, the
% inner cells +/-13 deg) plus a little noise. The line format exactly mirrors
% sweep_test.cpp so the real PuTTY capture parses identically.

    if nargin < 1 || isempty(outfile)
        outfile = fullfile(fileparts(mfilename('fullpath')), 'sample_sweep_log.txt');
    end

    rng(0);  % reproducible

    distances = [20 40 60 80 100];     % cm
    rel       = (-60:60)';             % deg from centre (full +/-60 sweep)
    centre    = 78;                    % SERVO_CENTER

    sigmaO = 22; offSL = -5; offSR =  5;   % outer (flat) cells
    sigmaI = 16; offL  = 13; offR  = -13;  % inner (angled) cells
    floorV = 0.03;

    fid = fopen(outfile, 'w');
    if fid < 0, error('make_sample_log:open', 'cannot write %s', outfile); end
    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid, '=~=~= PuTTY log (SYNTHETIC sample, not measured) =~=~=\n');
    fprintf(fid, '========================================\n');
    fprintf(fid, ' OUTSIDE-PAIR PHOTOTRANSISTOR SWEEP TEST\n');
    fprintf(fid, '========================================\n');

    for d = distances
        Ao = 2.6 * (20/d)^2;   % outer amplitude (inverse-square ref @ 20 cm)
        Ai = 1.8 * (20/d)^2;   % inner amplitude

        fprintf(fid, 'Enter distance in cm, then press Enter (e.g. 40):\n');
        fprintf(fid, '# SWEEP START dist=%.2f cm\n', d);
        fprintf(fid, '# DATA,dist_cm,servo_deg,rel_deg,sl_v,l_v,r_v,sr_v\n');

        sl = Ao*exp(-((rel-offSL).^2)/(2*sigmaO^2)) + floorV + 0.008*randn(size(rel));
        sr = Ao*exp(-((rel-offSR).^2)/(2*sigmaO^2)) + floorV + 0.008*randn(size(rel));
        l  = Ai*exp(-((rel-offL ).^2)/(2*sigmaI^2)) + floorV + 0.008*randn(size(rel));
        r  = Ai*exp(-((rel-offR ).^2)/(2*sigmaI^2)) + floorV + 0.008*randn(size(rel));
        sl = max(sl,0); sr = max(sr,0); l = max(l,0); r = max(r,0);

        for i = 1:numel(rel)
            servo = centre + rel(i);
            fprintf(fid, 'DATA,%.2f,%d,%d,%.4f,%.4f,%.4f,%.4f\n', ...
                    d, servo, rel(i), sl(i), l(i), r(i), sr(i));
            % Inject one corrupted line mid-sweep to prove the parser skips it.
            if d == 40 && rel(i) == 0
                fprintf(fid, 'DATA,40,78,0,0.55,GARB\n');
            end
        end

        fprintf(fid, '# SWEEP END dist=%.2f cm\n', d);
    end

    fprintf('Wrote synthetic log: %s\n', outfile);
end
