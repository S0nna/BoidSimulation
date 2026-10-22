import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Boids flocking simulation (Reynolds): separation, alignment, cohesion,
 * plus mouse attraction/repulsion, obstacles, and a predator.
 *
 * Run:  javac BoidSimulation.java && java BoidSimulation
 *
 * Controls: move mouse = attract (hold left = repel), right click = add obstacle,
 * S = spawn 10 boids at the mouse, SPACE = pause, T = trails, P = predator,
 * R = reset, C = clear obstacles.
 */
public class BoidSimulation extends JPanel {

    // ---- tunable parameters (modified by sliders) ----
    static volatile double separationW = 1.6, alignmentW = 1.0, cohesionW = 1.0;
    static volatile double perception = 55, separationDist = 22;
    static volatile double maxSpeed = 3.2, maxForce = 0.07;

    static final class Obstacle {
        final double x, y, r;
        Obstacle(double x, double y, double r) { this.x = x; this.y = y; this.r = r; }
    }

    static final class Boid {
        double x, y, vx, vy, ax, ay;
        float hue;
        Boid(double x, double y, Random rnd) {
            this.x = x; this.y = y;
            double a = rnd.nextDouble() * Math.PI * 2;
            vx = Math.cos(a) * maxSpeed * 0.7;
            vy = Math.sin(a) * maxSpeed * 0.7;
            hue = 0.5f + rnd.nextFloat() * 0.15f;
        }
    }

    private final Random rnd = new Random();
    private final List<Boid> boids = new ArrayList<>();
    private final List<Obstacle> obstacles = new ArrayList<>();
    private Boid predator;
    private boolean paused, trails = true, usePredator;
    private int mouseX = -1, mouseY = -1;
    private boolean mouseDown;
    private int fps;

    // spatial hash grid for O(n) neighbour queries
    private int cols, rows, cell = 60;
    private int[] head = new int[0], next = new int[0];

    private BufferedImageHolder buffer = new BufferedImageHolder();

    private static final class BufferedImageHolder {
        java.awt.image.BufferedImage img;
    }

    public BoidSimulation() {
        setPreferredSize(new Dimension(1100, 720));
        setBackground(Color.BLACK);
        setFocusable(true);
        for (int i = 0; i < 400; i++) boids.add(new Boid(rnd.nextInt(1100), rnd.nextInt(720), rnd));

        MouseAdapter m = new MouseAdapter() {
            @Override public void mouseMoved(MouseEvent e) { mouseX = e.getX(); mouseY = e.getY(); }
            @Override public void mouseDragged(MouseEvent e) {
                mouseX = e.getX(); mouseY = e.getY();
            }
            @Override public void mousePressed(MouseEvent e) {
                if (SwingUtilities.isRightMouseButton(e))
                    obstacles.add(new Obstacle(e.getX(), e.getY(), 15 + rnd.nextInt(25)));
                else if (SwingUtilities.isLeftMouseButton(e)) mouseDown = true;
            }
            @Override public void mouseReleased(MouseEvent e) { mouseDown = false; }
            @Override public void mouseExited(MouseEvent e) { mouseX = mouseY = -1; }
        };
        addMouseListener(m);
        addMouseMotionListener(m);

        // Key bindings work anywhere in the window, even after a slider takes focus
        bind("SPACE", () -> paused = !paused);
        bind("T", () -> trails = !trails);
        bind("C", obstacles::clear);
        bind("R", () -> {
            boids.clear();
            for (int i = 0; i < 400; i++) boids.add(new Boid(rnd.nextInt(Math.max(getWidth(), 1)), rnd.nextInt(Math.max(getHeight(), 1)), rnd));
        });
        bind("P", () -> {
            usePredator = !usePredator;
            predator = usePredator ? new Boid(getWidth() / 2.0, getHeight() / 2.0, rnd) : null;
        });
        bind("S", () -> {
            int sx = mouseX >= 0 ? mouseX : getWidth() / 2, sy = mouseY >= 0 ? mouseY : getHeight() / 2;
            for (int i = 0; i < 10; i++) boids.add(new Boid(sx + rnd.nextInt(30) - 15, sy + rnd.nextInt(30) - 15, rnd));
        });

        long[] last = {System.nanoTime()};
        int[] frames = {0};
        new Timer(16, e -> {
            if (!paused) step();
            repaint();
            frames[0]++;
            long now = System.nanoTime();
            if (now - last[0] > 1_000_000_000L) { fps = frames[0]; frames[0] = 0; last[0] = now; }
        }).start();
    }

    private void bind(String key, Runnable action) {
        getInputMap(WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke(key), key);
        getActionMap().put(key, new AbstractAction() {
            @Override public void actionPerformed(ActionEvent e) { action.run(); }
        });
    }

    // ---------------- simulation ----------------
    private void buildGrid() {
        int w = Math.max(getWidth(), 1), h = Math.max(getHeight(), 1);
        cell = (int) Math.max(perception, 20);
        cols = w / cell + 1;
        rows = h / cell + 1;
        if (head.length != cols * rows) head = new int[cols * rows];
        if (next.length != boids.size()) next = new int[boids.size()];
        java.util.Arrays.fill(head, -1);
        for (int i = 0; i < boids.size(); i++) {
            Boid b = boids.get(i);
            int c = clamp((int) (b.x / cell), 0, cols - 1), r = clamp((int) (b.y / cell), 0, rows - 1);
            next[i] = head[r * cols + c];
            head[r * cols + c] = i;
        }
    }

    private static int clamp(int v, int lo, int hi) { return Math.max(lo, Math.min(hi, v)); }

    private void step() {
        final int w = getWidth(), h = getHeight();
        if (w <= 0 || h <= 0) return;
        buildGrid();
        final double per2 = perception * perception, sep2 = separationDist * separationDist;

        for (int i = 0; i < boids.size(); i++) {
            Boid b = boids.get(i);
            double sx = 0, sy = 0, avx = 0, avy = 0, cx = 0, cy = 0;
            int n = 0, sn = 0;
            int bc = clamp((int) (b.x / cell), 0, cols - 1), br = clamp((int) (b.y / cell), 0, rows - 1);
            for (int r = Math.max(0, br - 1); r <= Math.min(rows - 1, br + 1); r++) {
                for (int c = Math.max(0, bc - 1); c <= Math.min(cols - 1, bc + 1); c++) {
                    for (int j = head[r * cols + c]; j != -1; j = next[j]) {
                        if (j == i) continue;
                        Boid o = boids.get(j);
                        double dx = o.x - b.x, dy = o.y - b.y, d2 = dx * dx + dy * dy;
                        if (d2 > per2 || d2 == 0) continue;
                        avx += o.vx; avy += o.vy;
                        cx += o.x; cy += o.y;
                        n++;
                        if (d2 < sep2) { sx -= dx / d2; sy -= dy / d2; sn++; }
                    }
                }
            }
            b.ax = 0; b.ay = 0;
            if (n > 0) {
                // alignment
                steer(b, avx / n, avy / n, alignmentW, false);
                // cohesion: steer toward the neighbours' centre
                steer(b, cx / n - b.x, cy / n - b.y, cohesionW, false);
            }
            if (sn > 0) steer(b, sx, sy, separationW, false);

            // mouse interaction
            if (mouseX >= 0) {
                double dx = mouseX - b.x, dy = mouseY - b.y, d = Math.hypot(dx, dy);
                if (d < 180 && d > 0) {
                    double s = mouseDown ? -2.5 : 0.6;
                    steer(b, dx, dy, s, false);
                }
            }
            // obstacle avoidance
            for (Obstacle o : obstacles) {
                double dx = b.x - o.x, dy = b.y - o.y, d = Math.hypot(dx, dy), range = o.r + 45;
                if (d < range && d > 0) steer(b, dx, dy, 3.0 * (range - d) / range, false);
            }
            // flee predator
            if (predator != null) {
                double dx = b.x - predator.x, dy = b.y - predator.y, d = Math.hypot(dx, dy);
                if (d < 140 && d > 0) steer(b, dx, dy, 3.5 * (140 - d) / 140, true);
            }
            // soft wall steering (boids turn around rather than wrap)
            double m = 60, t = 0.25;
            if (b.x < m) b.ax += t * (m - b.x) / m * maxForce * 4;
            if (b.x > w - m) b.ax -= t * (b.x - (w - m)) / m * maxForce * 4;
            if (b.y < m) b.ay += t * (m - b.y) / m * maxForce * 4;
            if (b.y > h - m) b.ay -= t * (b.y - (h - m)) / m * maxForce * 4;
        }

        for (Boid b : boids) integrate(b, w, h, maxSpeed);

        if (predator != null) updatePredator(w, h);
    }

    /** Reynolds steering: desired velocity minus current velocity, limited to maxForce. */
    private void steer(Boid b, double dx, double dy, double weight, boolean panic) {
        double len = Math.hypot(dx, dy);
        if (len == 0) return;
        double speed = panic ? maxSpeed * 1.4 : maxSpeed;
        double fx = dx / len * speed - b.vx, fy = dy / len * speed - b.vy;
        double fl = Math.hypot(fx, fy), lim = maxForce * (panic ? 2 : 1);
        if (fl > lim) { fx = fx / fl * lim; fy = fy / fl * lim; }
        b.ax += fx * weight;
        b.ay += fy * weight;
    }

    private void integrate(Boid b, int w, int h, double limit) {
        b.vx += b.ax; b.vy += b.ay;
        double sp = Math.hypot(b.vx, b.vy);
        double min = limit * 0.4;
        if (sp > limit * 1.4) { b.vx = b.vx / sp * limit * 1.4; b.vy = b.vy / sp * limit * 1.4; }
        else if (sp < min && sp > 0) { b.vx = b.vx / sp * min; b.vy = b.vy / sp * min; }
        b.x += b.vx; b.y += b.vy;
        // hard bounds as a safety net
        if (b.x < 0) { b.x = 0; b.vx = Math.abs(b.vx); }
        if (b.x > w) { b.x = w; b.vx = -Math.abs(b.vx); }
        if (b.y < 0) { b.y = 0; b.vy = Math.abs(b.vy); }
        if (b.y > h) { b.y = h; b.vy = -Math.abs(b.vy); }
        for (Obstacle o : obstacles) {
            double dx = b.x - o.x, dy = b.y - o.y, d = Math.hypot(dx, dy);
            if (d < o.r && d > 0) { b.x = o.x + dx / d * o.r; b.y = o.y + dy / d * o.r; }
        }
        // colour by heading
        b.hue = (float) ((Math.atan2(b.vy, b.vx) / (Math.PI * 2) + 1) % 1.0) * 0.35f + 0.5f;
    }

    private void updatePredator(int w, int h) {
        Boid p = predator;
        Boid target = null;
        double best = Double.MAX_VALUE;
        for (Boid b : boids) {
            double d = (b.x - p.x) * (b.x - p.x) + (b.y - p.y) * (b.y - p.y);
            if (d < best) { best = d; target = b; }
        }
        p.ax = p.ay = 0;
        if (target != null) steer(p, target.x - p.x, target.y - p.y, 1.0, false);
        integrate(p, w, h, maxSpeed * 1.05);
    }

    // ---------------- rendering ----------------
    @Override protected void paintComponent(Graphics g0) {
        int w = getWidth(), h = getHeight();
        if (w <= 0 || h <= 0) return;
        if (buffer.img == null || buffer.img.getWidth() != w || buffer.img.getHeight() != h) {
            buffer.img = new java.awt.image.BufferedImage(w, h, java.awt.image.BufferedImage.TYPE_INT_RGB);
            Graphics2D init = buffer.img.createGraphics();
            init.setColor(new Color(8, 10, 22));
            init.fillRect(0, 0, w, h);
            init.dispose();
        }
        Graphics2D g = buffer.img.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        // translucent fill produces motion trails
        g.setColor(trails ? new Color(8, 10, 22, 55) : new Color(8, 10, 22));
        g.fillRect(0, 0, w, h);

        g.setColor(new Color(70, 80, 110));
        for (Obstacle o : obstacles) g.fillOval((int) (o.x - o.r), (int) (o.y - o.r), (int) (o.r * 2), (int) (o.r * 2));

        for (Boid b : boids) drawBoid(g, b, Color.getHSBColor(b.hue, 0.75f, 1f), 7);
        if (predator != null) drawBoid(g, predator, new Color(255, 70, 70), 13);

        g.setColor(new Color(255, 255, 255, 200));
        g.setFont(new Font("SansSerif", Font.PLAIN, 12));
        g.drawString("Boids: " + boids.size() + "   FPS: " + fps + (paused ? "   [PAUSED]" : ""), 10, 18);
        g.drawString("Mouse: attract | Hold LMB: repel | RMB: obstacle | S: spawn | SPACE pause | T trails | P predator | R reset | C clear", 10, 34);
        g.dispose();
        g0.drawImage(buffer.img, 0, 0, null);
    }

    private void drawBoid(Graphics2D g, Boid b, Color c, double size) {
        double a = Math.atan2(b.vy, b.vx), ca = Math.cos(a), sa = Math.sin(a);
        Path2D.Double p = new Path2D.Double();
        p.moveTo(b.x + ca * size, b.y + sa * size);
        p.lineTo(b.x + (-ca * size * 0.7) - sa * size * 0.45, b.y + (-sa * size * 0.7) + ca * size * 0.45);
        p.lineTo(b.x + (-ca * size * 0.35), b.y + (-sa * size * 0.35));
        p.lineTo(b.x + (-ca * size * 0.7) + sa * size * 0.45, b.y + (-sa * size * 0.7) - ca * size * 0.45);
        p.closePath();
        g.setColor(c);
        g.fill(p);
    }

    // ---------------- UI ----------------
    private static JPanel slider(String name, int min, int max, int val, java.util.function.IntConsumer onChange) {
        JPanel p = new JPanel(new BorderLayout());
        p.setBackground(new Color(25, 28, 40));
        JLabel l = new JLabel(name + ": " + val);
        l.setForeground(Color.WHITE);
        JSlider s = new JSlider(min, max, val);
        s.setBackground(new Color(25, 28, 40));
        s.setFocusable(false);
        s.addChangeListener(e -> { l.setText(name + ": " + s.getValue()); onChange.accept(s.getValue()); });
        p.add(l, BorderLayout.NORTH);
        p.add(s, BorderLayout.CENTER);
        return p;
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame f = new JFrame("Boids Flocking Simulation");
            BoidSimulation sim = new BoidSimulation();

            JPanel controls = new JPanel(new GridLayout(1, 0, 8, 0));
            controls.setBackground(new Color(25, 28, 40));
            controls.setBorder(BorderFactory.createEmptyBorder(4, 8, 4, 8));
            controls.add(slider("Separation x10", 0, 40, (int) (separationW * 10), v -> separationW = v / 10.0));
            controls.add(slider("Alignment x10", 0, 40, (int) (alignmentW * 10), v -> alignmentW = v / 10.0));
            controls.add(slider("Cohesion x10", 0, 40, (int) (cohesionW * 10), v -> cohesionW = v / 10.0));
            controls.add(slider("Perception", 20, 150, (int) perception, v -> perception = v));
            controls.add(slider("Max speed x10", 10, 80, (int) (maxSpeed * 10), v -> maxSpeed = v / 10.0));

            f.setLayout(new BorderLayout());
            f.add(sim, BorderLayout.CENTER);
            f.add(controls, BorderLayout.SOUTH);
            f.pack();
            f.setLocationRelativeTo(null);
            f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            f.setVisible(true);
            sim.requestFocusInWindow();
        });
    }
}
