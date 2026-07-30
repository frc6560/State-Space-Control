package frc.robot.subsystems.stateSpaceSuperstructure.turret;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.ActionEvent;
import java.util.function.Consumer;
import javax.swing.AbstractAction;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.KeyStroke;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/** Desktop visualization and keyboard input for the simulated continuous turret. */
final class TurretSimulationWindow {
  enum Mode { PRESET, MANUAL, TARGET_TRACKING }

  record ControlInput(Mode mode, double manualRateDegreesPerSecond, double targetX, double targetY) {}

  private static final double TARGET_SPEED_METERS_PER_SECOND = 0.8;
  private final TurretPanel panel = new TurretPanel();

  TurretSimulationWindow(Consumer<Integer> presetSelector) {
    panel.presetSelector = presetSelector;
    bindKeys();
    SwingUtilities.invokeLater(() -> {
      JFrame frame = new JFrame("State-Space Turret Simulation");
      frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
      frame.setContentPane(panel);
      frame.setSize(560, 500);
      frame.setLocationByPlatform(true);
      frame.setVisible(true);
    });
  }

  ControlInput getControlInput() {
    double dx = (panel.rightHeld ? 1.0 : 0.0) - (panel.leftHeld ? 1.0 : 0.0);
    double dy = (panel.upHeld ? 1.0 : 0.0) - (panel.downHeld ? 1.0 : 0.0);
    if (dx != 0.0 || dy != 0.0) {
      panel.targetX += dx * TARGET_SPEED_METERS_PER_SECOND * 0.020;
      panel.targetY += dy * TARGET_SPEED_METERS_PER_SECOND * 0.020;
      panel.targetX = clamp(panel.targetX, -1.5, 1.5);
      panel.targetY = clamp(panel.targetY, -1.5, 1.5);
      panel.repaint();
      return new ControlInput(Mode.TARGET_TRACKING, 0.0, panel.targetX, panel.targetY);
    }
    if (panel.manualDirection != 0) {
      return new ControlInput(Mode.MANUAL, panel.manualDirection * 120.0,
          panel.targetX, panel.targetY);
    }
    return new ControlInput(panel.activeMode, 0.0, panel.targetX, panel.targetY);
  }

  void setTurretAngleRadians(double angleRadians) {
    panel.turretAngleRadians = angleRadians;
    panel.repaint();
  }

  void setGoalAngleRadians(double angleRadians) {
    panel.goalAngleRadians = angleRadians;
  }

  void setControlMode(String mode) {
    panel.modeText = mode;
  }

  void setPresetMode() {
    panel.activeMode = Mode.PRESET;
  }

  private void bindKeys() {
    bindPreset("1", 1);
    bindPreset("2", 2);
    bindPreset("3", 3);

    bindHeld("LEFT", () -> { panel.manualDirection = 1; panel.activeMode = Mode.MANUAL; },
        () -> panel.manualDirection = 0);
    bindHeld("RIGHT", () -> { panel.manualDirection = -1; panel.activeMode = Mode.MANUAL; },
        () -> panel.manualDirection = 0);
    bindHeld("W", () -> { panel.upHeld = true; panel.activeMode = Mode.TARGET_TRACKING; },
        () -> panel.upHeld = false);
    bindHeld("A", () -> { panel.leftHeld = true; panel.activeMode = Mode.TARGET_TRACKING; },
        () -> panel.leftHeld = false);
    bindHeld("S", () -> { panel.downHeld = true; panel.activeMode = Mode.TARGET_TRACKING; },
        () -> panel.downHeld = false);
    bindHeld("D", () -> { panel.rightHeld = true; panel.activeMode = Mode.TARGET_TRACKING; },
        () -> panel.rightHeld = false);
  }

  private void bindPreset(String key, int preset) {
    bindAction("pressed " + key, "preset-" + key,
        () -> { panel.activeMode = Mode.PRESET; panel.presetSelector.accept(preset); });
  }

  private void bindHeld(String key, Runnable pressed, Runnable released) {
    bindAction("pressed " + key, "pressed-" + key, pressed);
    bindAction("released " + key, "released-" + key, released);
  }

  private void bindAction(String stroke, String name, Runnable action) {
    panel.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(KeyStroke.getKeyStroke(stroke), name);
    panel.getActionMap().put(name, new AbstractAction() {
      @Override
      public void actionPerformed(ActionEvent event) {
        action.run();
      }
    });
  }

  private static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(high, value));
  }

  private static final class TurretPanel extends JPanel {
    private double turretAngleRadians;
    private double goalAngleRadians;
    private double targetX = 0.95;
    private double targetY = 0.35;
    private String modeText = "PRESET";
    private Mode activeMode = Mode.PRESET;
    private int manualDirection;
    private boolean upHeld, leftHeld, downHeld, rightHeld;
    private Consumer<Integer> presetSelector = ignored -> { };

    TurretPanel() {
      setBackground(new Color(28, 32, 38));
      presetSelector = ignored -> { };
    }

    @Override
    protected void paintComponent(Graphics graphics) {
      super.paintComponent(graphics);
      Graphics2D g = (Graphics2D) graphics.create();
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      int cx = getWidth() / 2;
      int cy = getHeight() / 2 + 25;
      int scale = Math.min(getWidth(), getHeight()) / 3;
      int turretX = cx + (int) (scale * 0.85 * Math.cos(turretAngleRadians));
      int turretY = cy - (int) (scale * 0.85 * Math.sin(turretAngleRadians));
      int targetPixelX = cx + (int) (scale * targetX);
      int targetPixelY = cy - (int) (scale * targetY);

      g.setColor(new Color(75, 82, 92));
      g.drawLine(cx - scale, cy, cx + scale, cy);
      g.drawLine(cx, cy - scale, cx, cy + scale);
      g.setColor(new Color(74, 172, 255));
      g.setStroke(new BasicStroke(12, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.drawLine(cx, cy, turretX, turretY);
      g.setColor(new Color(255, 92, 92));
      g.fillRect(targetPixelX - 12, targetPixelY - 12, 24, 24);
      g.setColor(Color.WHITE);
      g.fillOval(cx - 10, cy - 10, 20, 20);
      g.setColor(new Color(255, 220, 80));
      g.setStroke(new BasicStroke(2));
      int goalX = cx + (int) (scale * 0.95 * Math.cos(goalAngleRadians));
      int goalY = cy - (int) (scale * 0.95 * Math.sin(goalAngleRadians));
      g.drawLine(cx, cy, goalX, goalY);
      g.drawString("Mode: " + modeText, 16, 25);
      g.drawString("1/2/3 presets | Left/Right manual | WASD move target", 16, 48);
      g.drawString("Red = target block, blue = turret, yellow = goal", 16, 70);
      g.dispose();
    }
  }
}
