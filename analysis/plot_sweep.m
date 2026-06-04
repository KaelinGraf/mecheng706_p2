function [M, udist] = plot_sweep(logfile, savedir)
% PLOT_SWEEP  Parse and plot an outer-pair and inner-pair phototransistor sweep log.
%
%   plot_sweep()                  parses ../putty.log (relative to this file)
%   plot_sweep(logfile)           parses the given PuTTY capture
%   plot_sweep(logfile, savedir)  also writes PNGs into savedir
%   [M, udist] = plot_sweep(...)  returns the parsed matrix and distances

    if nargin < 1 || isempty(logfile)
        here = fileparts(mfilename('fullpath'));
        logfile = fullfile(here, 'putty.log');
    end
    if nargin < 2
        savedir = '';
    end
    if exist(logfile, 'file') ~= 2
        error('plot_sweep:noFile', 'Log file not found: %s', logfile);
    end

    close all;
    set(0, 'DefaultFigureWindowStyle', 'docked');

    % ---- Parse -----------------------------------------------------------
    txt = fileread(logfile);
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

    % ---- Figure 1: Outer pair angular response by distance ---------------
    f1 = figure('Name', 'Outer pair vs angle', 'Color', 'w');
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

    % ---- Figure 2: Heatmaps (Inner 0-70cm, Outer 50cm-max) ---------------
    f2 = figure('Name', 'Heatmaps', 'Color', 'w');
    
    relGrid = (max(min(rel), -180):1:min(max(rel),180))';
    gridSL = nan(numel(relGrid), nd);
    gridSR = nan(numel(relGrid), nd);
    gridL  = nan(numel(relGrid), nd);
    gridR  = nan(numel(relGrid), nd);
    
    for k = 1:nd
        m = (dist == udist(k));
        [rk, ord] = unique(rel(m));
        slk = sl(m); slk = slk(ord);
        srk = sr(m); srk = srk(ord);
        lk  = l(m);  lk  = lk(ord);
        rk_val = r(m); rk_val = rk_val(ord);
        
        if numel(rk) >= 2
            gridSL(:,k) = interp1(rk, slk, relGrid, 'linear', NaN);
            gridSR(:,k) = interp1(rk, srk, relGrid, 'linear', NaN);
            gridL(:,k)  = interp1(rk, lk,  relGrid, 'linear', NaN);
            gridR(:,k)  = interp1(rk, rk_val, relGrid, 'linear', NaN);
        end
    end
    
    max_dist = max(udist);

    % (a) L inner heatmap (0 - 70cm)
    axA = subplot(2,2,1);
    imagesc(axA, udist, relGrid, gridL, 'AlphaData', ~isnan(gridL));
    set(axA, 'YDir', 'normal'); colorbar(axA); xticks(axA, udist);
    xlim(axA, [0 70]);
    title(axA, 'L (Inner-Left) over angle x distance');
    xlabel(axA, 'distance (cm)'); ylabel(axA, 'angle from centre (deg)');
    
    % (b) R inner heatmap (0 - 70cm)
    axB = subplot(2,2,2);
    imagesc(axB, udist, relGrid, gridR, 'AlphaData', ~isnan(gridR));
    set(axB, 'YDir', 'normal'); colorbar(axB); xticks(axB, udist);
    xlim(axB, [0 70]);
    title(axB, 'R (Inner-Right) over angle x distance');
    xlabel(axB, 'distance (cm)'); ylabel(axB, 'angle from centre (deg)');
    
    % (c) SL outer heatmap (50cm - max dist)
    axC = subplot(2,2,3);
    imagesc(axC, udist, relGrid, gridSL, 'AlphaData', ~isnan(gridSL));
    set(axC, 'YDir', 'normal'); colorbar(axC); xticks(axC, udist);
    xlim(axC, [50 max_dist]);
    title(axC, 'SL (Outer-Left) over angle x distance');
    xlabel(axC, 'distance (cm)'); ylabel(axC, 'angle from centre (deg)');
    
    % (d) SR outer heatmap (50cm - max dist)
    axD = subplot(2,2,4);
    imagesc(axD, udist, relGrid, gridSR, 'AlphaData', ~isnan(gridSR));
    set(axD, 'YDir', 'normal'); colorbar(axD); xticks(axD, udist);
    xlim(axD, [50 max_dist]);
    title(axD, 'SR (Outer-Right) over angle x distance');
    xlabel(axD, 'distance (cm)'); ylabel(axD, 'angle from centre (deg)');
    
    sgtitle(f2, 'Phototransistor 2-D Maps (Inner: 0-70cm, Outer: 50cm-Max)');

    % ---- Figure 3: All four cells at 4 specific distances ----------------
    f3 = figure('Name', '4 Distances Comparison', 'Color', 'w');
    
    % Indices: Closest (1), 4th, 8th, Furthest (nd). Safe limits applied.
    d_idx = [1, min(4, nd), min(8, nd), nd]; 
    
    for i = 1:4
        ax = subplot(2,2,i); hold(ax,'on'); grid(ax,'on');
        k = d_idx(i);
        
        m = (dist == udist(k));
        [rk, ord] = sort(rel(m));
        
        vsl = sl(m); vsl = vsl(ord);
        vl  = l(m);  vl  = vl(ord);
        vr  = r(m);  vr  = vr(ord);
        vsr = sr(m); vsr = vsr(ord);
        
        plot(ax, rk, vsl, '-',  'LineWidth',1.5, 'Color',[0.85 0.20 0.20]);
        plot(ax, rk, vl,  '--', 'LineWidth',1.3, 'Color',[0.20 0.50 0.20]);
        plot(ax, rk, vr,  '--', 'LineWidth',1.3, 'Color',[0.20 0.20 0.85]);
        plot(ax, rk, vsr, '-',  'LineWidth',1.5, 'Color',[0.95 0.55 0.10]);
        
        xline(ax, 0, ':', 'Color',[.5 .5 .5]);
        title(ax, sprintf('All four cells @ %g cm', udist(k)));
        xlabel(ax, 'turret angle from centre (deg)'); ylabel(ax, 'voltage (V)');
        
        if i == 1
            legend(ax, {'SL outer','L inner','R inner','SR outer'}, ...
                   'Location','northeast','Box','off');
        end
    end
    sgtitle(f3, 'All sensors shape comparison at selected distances');

    % ---- Figure 4: Inner pair angular response by distance ---------------
    f4 = figure('Name', 'Outer pair vs angle', 'Color', 'w');
    ax1b = subplot(2,2,1); hold(ax1b,'on'); grid(ax1b,'on');
    ax2b = subplot(2,2,2); hold(ax2b,'on'); grid(ax2b,'on');
    ax3b = subplot(2,2,3); hold(ax3b,'on'); grid(ax3b,'on');
    ax4b = subplot(2,2,4); hold(ax4b,'on'); grid(ax4b,'on');
    
    for k = 1:nd
        m = (dist == udist(k));
        [rk, ord] = sort(rel(m));
        lk = l(m); lk = lk(ord);
        rk_val = r(m); rk_val = rk_val(ord);
        c = cmap(k,:);
        
        plot(ax1b, rk, lk,             '-', 'Color', c, 'LineWidth', 1.3);
        plot(ax2b, rk, rk_val,         '-', 'Color', c, 'LineWidth', 1.3);
        plot(ax3b, rk, lk - rk_val,    '-', 'Color', c, 'LineWidth', 1.3);
        plot(ax4b, rk, lk + rk_val,    '-', 'Color', c, 'LineWidth', 1.3);
    end
    
    title(ax1b, 'L (inner-left) vs angle');
    title(ax2b, 'R (inner-right) vs angle');
    title(ax3b, 'Inner difference  L - R  (steering signal)');
    title(ax4b, 'Inner sum  L + R  (magnitude)');
    
    for ax = [ax1b ax2b ax3b ax4b]
        xlabel(ax, 'turret angle from centre (deg)');
        ylabel(ax, 'filtered voltage (V)');
        xline(ax, 0, ':', 'boresight', 'LabelOrientation','horizontal', ...
              'LabelVerticalAlignment','bottom', 'Color',[.5 .5 .5]);
    end
    yline(ax3b, 0, ':', 'Color', [.5 .5 .5]);
    legend(ax1b, distLabels, 'Location', 'northeast', 'Box', 'off');
    sgtitle(f4, 'Inner (angled) phototransistor pair - angular response by distance');

    % ---- Optional save ---------------------------------------------------
    if ~isempty(savedir)
        if exist(savedir, 'dir') ~= 7
            mkdir(savedir);
        end
        exportgraphics(f1, fullfile(savedir, 'sweep_outer_angle.png'), 'Resolution', 150);
        exportgraphics(f2, fullfile(savedir, 'sweep_maps.png'),  'Resolution', 150);
        exportgraphics(f3, fullfile(savedir, 'sweep_4distances.png'), 'Resolution', 150);
        exportgraphics(f4, fullfile(savedir, 'sweep_inner_angle.png'), 'Resolution', 150);
        fprintf('Saved figures to %s\n', savedir);
    end
end