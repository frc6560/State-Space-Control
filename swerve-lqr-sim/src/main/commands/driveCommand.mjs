/** Driver command/input adapter. Hardware projects can replace this with their controller binding. */
export function readDriverInput(gamepads, pressed) {
  const gamepad = [...gamepads].find(Boolean);
  if (gamepad) return { translationX: gamepad.axes[0] ?? 0, translationY: -(gamepad.axes[1] ?? 0), omega: gamepad.axes[2] ?? 0 };
  return {
    translationX: (pressed.has("KeyD") ? 1 : 0) - (pressed.has("KeyA") ? 1 : 0),
    translationY: (pressed.has("KeyW") ? 1 : 0) - (pressed.has("KeyS") ? 1 : 0),
    omega: (pressed.has("ArrowLeft") ? 1 : 0) - (pressed.has("ArrowRight") ? 1 : 0)
  };
}
