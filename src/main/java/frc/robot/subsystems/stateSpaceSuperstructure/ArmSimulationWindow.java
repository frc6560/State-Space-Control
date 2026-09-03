package frc.robot.subsystems.stateSpaceSuperstructure;

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

/** A small desktop-only window that makes the simulated arm motion immediately visible. */
final class ArmSimulationWindow {
  private final ArmPanel panel = new ArmPanel();

  ArmSimulationWindow(Consumer<ArmSubsystem.TargetPosition> selectTarget) {
    bindPresetKeys(selectTarget);
    SwingUtilities.invokeLater(() -> {
      JFrame frame = new JFrame("State-Space Arm Simulation");
      frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
      frame.setContentPane(panel);
      frame.setSize(500, 400);
      frame.setLocationByPlatform(true);
      frame.setVisible(true);
    });
  }

  void setAngleRadians(double angleRadians) {
    panel.setAngleRadians(angleRadians);
  }

  private void bindPresetKeys(Consumer<ArmSubsystem.TargetPosition> selectTarget) {
    bindPresetKey("1", ArmSubsystem.TargetPosition.STOWED, selectTarget);
    bindPresetKey("2", ArmSubsystem.TargetPosition.INTAKE, selectTarget);
    bindPresetKey("3", ArmSubsystem.TargetPosition.SCORE, selectTarget);
    bindPresetKey("NUMPAD1", ArmSubsystem.TargetPosition.STOWED, selectTarget);
    bindPresetKey("NUMPAD2", ArmSubsystem.TargetPosition.INTAKE, selectTarget);
    bindPresetKey("NUMPAD3", ArmSubsystem.TargetPosition.SCORE, selectTarget);
  }

  private void bindPresetKey(String key, ArmSubsystem.TargetPosition target,
      Consumer<ArmSubsystem.TargetPosition> selectTarget) {
    String actionName = "select-" + key;
    panel.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW)
        .put(KeyStroke.getKeyStroke("pressed " + key), actionName);
    panel.getActionMap().put(actionName, new AbstractAction() {
      @Override
      public void actionPerformed(ActionEvent event) {
        selectTarget.accept(target);
      }
    });
  }

  private static final class ArmPanel extends JPanel {
    private volatile double angleRadians;

    ArmPanel() {
      setBackground(new Color(30, 34, 40));
    }

    void setAngleRadians(double angleRadians) {
      this.angleRadians = angleRadians;
      SwingUtilities.invokeLater(this::repaint);
    }

    @Override
    protected void paintComponent(Graphics graphics) {
      super.paintComponent(graphics);
      Graphics2D g = (Graphics2D) graphics.create();
      g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      int pivotX = getWidth() / 2;
      int pivotY = getHeight() - 55;
      int armPixels = Math.min(getWidth(), getHeight()) - 105;
      int endX = pivotX + (int) (armPixels * Math.cos(angleRadians));
      int endY = pivotY - (int) (armPixels * Math.sin(angleRadians));

      g.setColor(new Color(90, 96, 105));
      g.fillRect(0, pivotY, getWidth(), getHeight() - pivotY);
      g.setColor(new Color(247, 180, 45));
      g.setStroke(new BasicStroke(18, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
      g.drawLine(pivotX, pivotY, endX, endY);
      g.setColor(Color.WHITE);
      g.fillOval(pivotX - 12, pivotY - 12, 24, 24);
      g.drawString(String.format("Arm angle: %.1f degrees", Math.toDegrees(angleRadians)), 16, 25);
      g.drawString("Keyboard: 1 = stowed, 2 = intake, 3 = score", 16, 48);
      g.dispose();
    }
  }
}
