function occupancy_viz(mode, src, baud)
%OCCUPANCY_VIZ  Live viewer for the Arduino-streamed occupancy map (protocol v1).
%
%   occupancy_viz('serial', 'COM5', 115200)
%       Open the serial port, read available lines as they arrive, parse them
%       with parse_occupancy(), and redraw on every complete frame. Runs until
%       the figure window is closed (or Ctrl-C).
%
%   occupancy_viz('file', 'capture.log')
%       Replay a captured text log line-by-line (with a small pause) through the
%       very same parser + renderer, so a recorded run looks like a live one.
%
%   occupancy_viz('file', 'capture.log', fps)
%       As above but cap the replay at ~fps frames/sec (default 4).
%
%   Rendering (see OCCUPANCY_PROTOCOL.md for the wire contract):
%     * occupancy heatmap from the raw log-odds, diverging colormap
%       (blue = free/neg, white = unknown/0, red = occupied/pos), clamped to
%       the protocol's [-12, +12], drawn in WORLD coordinates (cm) using the
%       window origin and cell size;
%     * robot marker at (pose_x, pose_y) with a heading arrow (pose_th);
%     * fire marker at (fire_x, fire_y) ONLY when fire_conf > 0, its size/alpha
%       scaled by confidence;
%     * the goal cell highlighted when goal >= 0;
%     * axis equal, cm axis labels, a title showing seq, measured fps, and the
%       robot's (dead-reckoned) linear & angular speed, updated in place.
%
%   This function only consumes the protocol; it never drives the robot.

    if nargin < 1 || isempty(mode)
        error('occupancy_viz:args', ...
            'Usage: occupancy_viz(''serial'',port,baud) or occupancy_viz(''file'',path[,fps])');
    end

    switch lower(mode)
        case 'serial'
            if nargin < 2 || isempty(src)
                error('occupancy_viz:args', 'serial mode needs a port, e.g. ''COM5''.');
            end
            if nargin < 3 || isempty(baud)
                baud = 115200;
            end
            run_serial(src, baud);

        case 'file'
            if nargin < 2 || isempty(src)
                error('occupancy_viz:args', 'file mode needs a log path.');
            end
            if nargin < 3 || isempty(baud)
                fps = 4;                 % default replay rate
            else
                fps = baud;              % third arg doubles as replay fps here
            end
            run_file(src, fps);

        otherwise
            error('occupancy_viz:mode', 'Unknown mode ''%s'' (use ''serial'' or ''file'').', mode);
    end
end

% ============================ run modes ==================================

function run_serial(port, baud)
    % Live serial (NON-BLOCKING): a terminator callback renders each frame as it
    % arrives, so this returns immediately and MATLAB stays responsive. Close the
    % figure window to stop streaming and release the port.
    sp = serialport(port, baud);                 % MATLAB base serialport
    configureTerminator(sp, "LF");               % lines are \n-terminated

    parser = parse_occupancy();
    h = local_initFigure(sprintf('Occupancy (serial %s @ %d)', port, baud));
    lastT = tic;
    fps   = 0;

    % Fire onLine() for every \n-terminated line that arrives, and tear the port
    % down when the figure is closed.
    configureCallback(sp, "terminator", @(src, ~) onLine(src));
    set(h.fig, 'DeleteFcn', @(~, ~) stopSerial());
    fprintf('occupancy_viz: listening on %s @ %d (close the figure to stop)\n', ...
        port, baud);

    % --- nested callbacks (share sp/parser/h/lastT/fps) -------------------
    function onLine(src)
        % Drain everything queued and render only the NEWEST complete frame, so a
        % slow draw never lags behind the robot.
        latest = [];
        nLines = 0;
        while src.NumBytesAvailable > 0 && nLines < 4000
            ln = readline(src);
            nLines = nLines + 1;
            if ismissing(ln), break; end
            f = parser.feed(ln);
            if ~isempty(f), latest = f; end      % overwrite -> newest wins
        end
        if ~isempty(latest) && ishandle(h.fig)
            dt = toc(lastT); lastT = tic;
            if dt > 0, fps = 0.7 * fps + 0.3 * (1 / dt); end
            local_render(h, latest, fps, dt);
        end
    end

    function stopSerial()
        try
            configureCallback(sp, "off");
        catch
        end
        local_closePort(sp);
    end
end

function run_file(logpath, fps)
    % File replay: feed a captured log line-by-line through the same parser.
    if exist(logpath, 'file') ~= 2
        error('occupancy_viz:noFile', 'Log file not found: %s', logpath);
    end
    if ~isfinite(fps) || fps <= 0
        fps = 4;
    end
    dwell = 1 / fps;

    txt   = fileread(logpath);
    lines = regexp(txt, '\r?\n', 'split');

    parser = parse_occupancy();
    h = local_initFigure(sprintf('Occupancy (replay %s)', logpath));

    lastT = tic;
    shown = 0;
    for i = 1:numel(lines)
        if ~ishandle(h.fig)
            break;                               % user closed the window
        end
        frame = parser.feed(lines{i});
        if ~isempty(frame)
            dt = toc(lastT); lastT = tic;
            measFps = 0;
            if dt > 0
                measFps = 1 / dt;
            end
            local_render(h, frame, measFps, dt);
            shown = shown + 1;
            pause(dwell);                        % pace the replay
        end
    end
    fprintf('occupancy_viz: replay finished, %d frame(s) rendered from %s\n', ...
        shown, logpath);
end

% ============================ rendering ==================================

function h = local_initFigure(name)
    % Create the figure/axes/graphics once; everything is updated in place.
    h.fig = figure('Name', name, 'Color', 'w', 'NumberTitle', 'off', ...
                   'Position', [120 90 760 720]);
    h.ax = axes('Parent', h.fig);
    hold(h.ax, 'on');
    axis(h.ax, 'equal');
    set(h.ax, 'YDir', 'normal');                 % world Y up
    grid(h.ax, 'on');
    box(h.ax, 'on');
    xlabel(h.ax, 'world X (cm)');
    ylabel(h.ax, 'world Y (cm)');
    colormap(h.ax, local_divergingMap(256));

    % Heatmap image placeholder (XData/YData/CData set per frame).
    h.img = imagesc(h.ax, 'CData', zeros(2, 2));
    uistack(h.img, 'bottom');

    h.cb = colorbar(h.ax);
    h.cb.Label.String = 'log-odds  (blue = free, white = unknown, red = occupied)';
    clim(h.ax, [-12 12]);                        % protocol clamp [-12, +12]

    % Goal cell highlight (a rectangle around the goal cell).
    h.goal = rectangle('Parent', h.ax, 'Position', [0 0 eps eps], ...
        'EdgeColor', [0 0.6 0], 'LineWidth', 2, 'Visible', 'off');

    % Breadcrumb trail of recent robot positions. animatedline auto-drops the
    % oldest points past MaximumNumPoints; in world coords, so the trail scrolls
    % off as the rolling window follows the robot. Drawn under the robot marker.
    h.trail = animatedline('Parent', h.ax, 'Color', [0.20 0.20 0.20], ...
        'LineStyle', '-', 'LineWidth', 1.0, 'MaximumNumPoints', 600);

    % Robot body + heading arrow (quiver), updated per frame.
    h.robot = plot(h.ax, NaN, NaN, 'o', 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.1 0.1 0.1], 'MarkerEdgeColor', 'w', 'LineWidth', 1);
    h.head = quiver(h.ax, NaN, NaN, 0, 0, 0, 'Color', [0.1 0.1 0.1], ...
        'LineWidth', 2, 'MaxHeadSize', 2, 'AutoScale', 'off');

    % Fire marker (scatter so we can scale size/alpha by confidence).
    h.fire = scatter(h.ax, NaN, NaN, 200, [0.9 0.3 0.0], 'filled', ...
        'MarkerEdgeColor', [0.6 0.1 0.0], 'MarkerFaceAlpha', 1, 'Visible', 'off');

    h.title = title(h.ax, 'waiting for frames...');
    drawnow;
end

function local_render(h, frame, fps, dt)
    % Update every graphics object from one completed frame. No new objects.
    if ~ishandle(h.fig)
        return;
    end
    n  = frame.n;
    cc = frame.cell_cm;

    % World extents of the cell grid centres. Cell (x, y) centre:
    %   wx = (org_cx + x + 0.5) * cell_cm,  wy = (org_cy + y + 0.5) * cell_cm.
    x0 = (frame.org(1) + 0.5) * cc;              % centre of column x = 0
    y0 = (frame.org(2) + 0.5) * cc;              % centre of row    y = 0
    x1 = (frame.org(1) + (n - 1) + 0.5) * cc;
    y1 = (frame.org(2) + (n - 1) + 0.5) * cc;

    % Heatmap: grid(y+1, x+1) already has row y in row index y+1, so with
    % YDir normal and ascending YData the image is the right way up.
    set(h.img, 'XData', [x0 x1], 'YData', [y0 y1], 'CData', frame.grid);

    % Keep the view framed on the window (+ half a cell border).
    pad = cc;
    xlim(h.ax, [x0 - pad, x1 + pad]);
    ylim(h.ax, [y0 - pad, y1 + pad]);

    % Robot marker + heading arrow.
    px = frame.pose(1); py = frame.pose(2); th = frame.pose(3);
    arrowLen = 3 * cc;                           % a few cells long
    set(h.robot, 'XData', px, 'YData', py);
    set(h.head, 'XData', px, 'YData', py, ...
        'UData', arrowLen * cos(th), 'VData', arrowLen * sin(th));
    addpoints(h.trail, px, py);                  % extend the breadcrumb trail

    % Robot speed from successive pose samples: the protocol carries pose, not
    % velocity, so differentiate world position over the measured frame interval
    % dt. This is the DEAD-RECKONED speed (integrated from commanded motion; no
    % wheel encoders), smoothed to tame the ~1 Hz frame jitter. Persisted across
    % calls because the per-frame h struct is passed by value.
    persistent prevX prevY prevTh spd angspd
    if isempty(prevX) || isempty(dt) || dt <= 0
        if isempty(spd), spd = 0; angspd = 0; end
    else
        instSpd = hypot(px - prevX, py - prevY) / dt;            % cm/s
        dth     = mod(th - prevTh + pi, 2*pi) - pi;              % wrapped heading delta
        instAng = abs(dth) / dt * 180/pi;                        % deg/s
        if isempty(spd),    spd    = instSpd; else, spd    = 0.6 * spd    + 0.4 * instSpd; end
        if isempty(angspd), angspd = instAng; else, angspd = 0.6 * angspd + 0.4 * instAng; end
    end
    prevX = px; prevY = py; prevTh = th;

    % Fire marker only when confidence > 0; scale size & alpha by confidence.
    conf = frame.fire(3);
    if conf > 0
        sz = 120 + 380 * min(conf, 1);           % 120 .. 500 pts^2
        set(h.fire, 'XData', frame.fire(1), 'YData', frame.fire(2), ...
            'SizeData', sz, 'MarkerFaceAlpha', max(0.25, min(conf, 1)), ...
            'Visible', 'on');
    else
        set(h.fire, 'Visible', 'off');
    end

    % Goal cell highlight when goal >= 0 (both indices must be valid).
    gx = frame.goal(1); gy = frame.goal(2);
    if gx >= 0 && gy >= 0 && gx < n && gy < n
        gwx = (frame.org(1) + gx) * cc;          % lower-left corner of the cell
        gwy = (frame.org(2) + gy) * cc;
        set(h.goal, 'Position', [gwx, gwy, cc, cc], 'Visible', 'on');
    else
        set(h.goal, 'Visible', 'off');
    end

    % Title with seq, measured fps, and dead-reckoned speed.
    set(h.title, 'String', sprintf( ...
        'seq %d   %dx%d   %.2f cm/cell   %.1f fps   |   v=%.1f cm/s   turn=%.0f deg/s', ...
        frame.seq, n, n, cc, fps, spd, angspd));

    drawnow limitrate;
end

% ============================ helpers ====================================

function cmap = local_divergingMap(m)
    % Blue -> white -> red diverging colormap (no toolbox dependency).
    if nargin < 1, m = 256; end
    half = floor(m / 2);
    % Blue (free) up to white (unknown).
    bw = [linspace(0.10, 1, half).', linspace(0.30, 1, half).', ones(half, 1)];
    % White (unknown) up to red (occupied).
    up = m - half;
    wr = [ones(up, 1), linspace(1, 0.20, up).', linspace(1, 0.15, up).'];
    cmap = [bw; wr];
end

function local_closePort(sp)
    % Best-effort release of the serialport handle on cleanup.
    try
        delete(sp);
    catch
        % port already gone / never opened -- nothing to do
    end
end
