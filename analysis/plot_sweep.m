function [M, udist] = plot_sweep(logfile, savedir)
% PLOT_SWEEP  Parse and plot an outer-pair phototransistor sweep log.
%
%   plot_sweep()                  parses ../putty.log (relative to this file)
%   plot_sweep(logfile)           parses the given PuTTY capture
%   plot_sweep(logfile, savedir)  also writes PNGs into savedir
%   [M, udist] = plot_sweep(...)  returns the parsed matrix and distances
%
% The companion firmware (sweep_test.cpp) emits one machine-readable row per
% turret angle:
%
%   DATA,dist_cm,servo_deg,rel_deg,sl_v,l_v,r_v,sr_v
%
% where sl,sr are the new OUTER flat-mounted pair under test, l,r are the
% inner angled pair (logged for context), and rel_deg = servo_deg - centre
% (0 deg = boresight, i.e. light dead ahead of the turret centre). With the
% light fixed ahead of centre and the turret sweeping, rel_deg equals the
% light's incidence angle on the flat outer cells, so each distance gives one
% angular-response curve.
%
% Only lines that match the strict DATA pattern are used, so PuTTY banners,
% prompts and the occasional Bluetooth glitch line are ignored automatically.

    if nargin < 1 || isempty(logfile)
        here = fileparts(mfilename('fullpath'));
        logfile = fullfile(here, '..', 'putty.log');
    end
    if nargin < 2
        savedir = '';
    end

    if exist(logfile, 'file') ~= 2
        error('plot_sweep:noFile', 'Log file not found: %s', logfile);
    end

    % ---- Parse -----------------------------------------------------------
    txt = fileread(logfile);

    % DATA,<dist>,<servo>,<rel>,<sl>,<l>,<r>,<sr>  (rel/servo are integers;
    % the rest may be decimals). Anchored per line so junk lines are skipped.
    pat = ['^DATA,(-?\d+(?:\.\d+)?),(-?\d+),(-?\d+),' ...
           '(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?),' ...
           '(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)\s*$'];
    tok = regexp(txt, pat, 'tokens', 'lineanchors');

    if isempty(tok)
        error('plot_sweep:noData', ...
            ['No DATA rows found in %s.\n' ...
             'Expected rows like: DATA,40,78,0,0.1234,0.3210,0.2980,0.1502'], ...
            logfile);
    end

    M = zeros(numel(tok), 7);
    for i = 1:numel(tok)
        M(i, :) = str2double(tok{i});
    end

    dist = M(:,1);  rel = M(:,3);
    sl   = M(:,4);  l = M(:,5);  r = M(:,6);  sr = M(:,7);

    udist = unique(dist);
    nd    = numel(udist);
    cmap  = parula(max(nd, 2));

    fprintf('plot_sweep: %d data rows, %d distance(s): %s cm\n', ...
        size(M,1), nd, strtrim(sprintf('%g ', udist)));
    fprintf('            rel angle range: %g to %g deg\n', min(rel), max(rel));

    distLabels = arrayfun(@(d) sprintf('%g cm', d), udist, 'uni', 0);

    % ---- Figure 1: outer pair angular response, family by distance -------
    f1 = figure('Name', 'Outer pair vs angle', 'Color', 'w', ...
                'Position', [80 80 1100 760]);

    ax1 = subplot(2,2,1); hold(ax1,'on'); grid(ax1,'on');
    ax2 = subplot(2,2,2); hold(ax2,'on'); grid(ax2,'on');
    ax3 = subplot(2,2,3); hold(ax3,'on'); grid(ax3,'on');
    ax4 = subplot(2,2,4); hold(ax4,'on'); grid(ax4,'on');

    for k = 1:nd
        m = (dist == udist(k));
        [rk, ord] = sort(rel(m));
        slk = sl(m); slk = slk(ord);
        srk = sr(m); srk = srk(ord);
        c = cmap(k,:);
        plot(ax1, rk, slk,        '-', 'Color', c, 'LineWidth', 1.3);
        plot(ax2, rk, srk,        '-', 'Color', c, 'LineWidth', 1.3);
        plot(ax3, rk, slk - srk,  '-', 'Color', c, 'LineWidth', 1.3);
        plot(ax4, rk, slk + srk,  '-', 'Color', c, 'LineWidth', 1.3);
    end

    title(ax1, 'SL (outer-left) vs angle');
    title(ax2, 'SR (outer-right) vs angle');
    title(ax3, 'Outer difference  SL - SR  (steering signal)');
    title(ax4, 'Outer sum  SL + SR  (magnitude)');
    for ax = [ax1 ax2 ax3 ax4]
        xlabel(ax, 'turret angle from centre (deg)');
        ylabel(ax, 'filtered voltage (V)');
        xline(ax, 0, ':', 'boresight', 'LabelOrientation','horizontal', ...
              'LabelVerticalAlignment','bottom', 'Color',[.5 .5 .5]);
    end
    yline(ax3, 0, ':', 'Color', [.5 .5 .5]);
    legend(ax1, distLabels, 'Location', 'northeast', 'Box', 'off');

    sgtitle(f1, 'Outer (flat) phototransistor pair - angular response by distance');

    % ---- Figure 2: inner-vs-outer per distance + range falloff -----------
    f2 = figure('Name', 'Inner vs outer / falloff', 'Color', 'w', ...
                'Position', [120 120 1100 760]);

    % (a) all four cells at the nearest distance, to compare shapes
    axA = subplot(2,2,1); hold(axA,'on'); grid(axA,'on');
    kNear = 1;  % udist is sorted ascending
    mN = (dist == udist(kNear));
    [rN, ordN] = sort(rel(mN));
    getc = @(v) v(ordN);
    vsl = sl(mN); vl = l(mN); vr = r(mN); vsr = sr(mN);
    pSL = plot(axA, rN, getc(vsl), '-',  'LineWidth',1.5, 'Color',[0.85 0.20 0.20]);
    pL  = plot(axA, rN, getc(vl),  '--', 'LineWidth',1.3, 'Color',[0.20 0.50 0.20]);
    pR  = plot(axA, rN, getc(vr),  '--', 'LineWidth',1.3, 'Color',[0.20 0.20 0.85]);
    pSR = plot(axA, rN, getc(vsr), '-',  'LineWidth',1.5, 'Color',[0.95 0.55 0.10]);
    xline(axA, 0, ':', 'Color',[.5 .5 .5]);
    title(axA, sprintf('All four cells @ %g cm', udist(kNear)));
    xlabel(axA, 'turret angle from centre (deg)'); ylabel(axA, 'voltage (V)');
    legend(axA, [pSL pL pR pSR], {'SL outer','L inner','R inner','SR outer'}, ...
           'Location','northeast','Box','off');

    % (b) peak outer voltage vs distance (range falloff)
    axB = subplot(2,2,2); hold(axB,'on'); grid(axB,'on');
    peakSL = zeros(nd,1); peakSR = zeros(nd,1);
    for k = 1:nd
        m = (dist == udist(k));
        peakSL(k) = max(sl(m));
        peakSR(k) = max(sr(m));
    end
    plot(axB, udist, peakSL, 'o-', 'LineWidth',1.4, 'Color',[0.85 0.20 0.20], ...
         'MarkerFaceColor',[0.85 0.20 0.20]);
    plot(axB, udist, peakSR, 's-', 'LineWidth',1.4, 'Color',[0.95 0.55 0.10], ...
         'MarkerFaceColor',[0.95 0.55 0.10]);
    title(axB, 'Peak outer-cell voltage vs distance');
    xlabel(axB, 'distance (cm)'); ylabel(axB, 'peak filtered voltage (V)');
    legend(axB, {'SL','SR'}, 'Location','northeast','Box','off');

    % (c,d) heatmaps of SL and SR over (angle, distance) on a common grid
    relGrid = (max(min(rel), -180):1:min(max(rel),180))';
    gridSL = nan(numel(relGrid), nd);
    gridSR = nan(numel(relGrid), nd);
    for k = 1:nd
        m = (dist == udist(k));
        [rk, ord] = unique(rel(m));      % unique+sorted angle axis
        slk = sl(m); slk = slk(ord);
        srk = sr(m); srk = srk(ord);
        if numel(rk) >= 2
            gridSL(:,k) = interp1(rk, slk, relGrid, 'linear', NaN);
            gridSR(:,k) = interp1(rk, srk, relGrid, 'linear', NaN);
        end
    end

    axC = subplot(2,2,3);
    imagesc(axC, udist, relGrid, gridSL, 'AlphaData', ~isnan(gridSL));
    set(axC, 'YDir', 'normal'); colorbar(axC); xticks(axC, udist);
    title(axC, 'SL (V) over angle x distance');
    xlabel(axC, 'distance (cm)'); ylabel(axC, 'angle from centre (deg)');

    axD = subplot(2,2,4);
    imagesc(axD, udist, relGrid, gridSR, 'AlphaData', ~isnan(gridSR));
    set(axD, 'YDir', 'normal'); colorbar(axD); xticks(axD, udist);
    title(axD, 'SR (V) over angle x distance');
    xlabel(axD, 'distance (cm)'); ylabel(axD, 'angle from centre (deg)');

    sgtitle(f2, 'Outer pair: shape comparison, range falloff, and 2-D maps');

    % ---- Optional save ---------------------------------------------------
    if ~isempty(savedir)
        if exist(savedir, 'dir') ~= 7
            mkdir(savedir);
        end
        exportgraphics(f1, fullfile(savedir, 'sweep_outer_angle.png'), 'Resolution', 150);
        exportgraphics(f2, fullfile(savedir, 'sweep_outer_maps.png'),  'Resolution', 150);
        fprintf('Saved figures to %s\n', savedir);
    end
end
