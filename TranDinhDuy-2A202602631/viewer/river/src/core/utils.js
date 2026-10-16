export const clamp = (n, low, high) => Math.min(high, Math.max(low, n));
export const copy = value => JSON.parse(JSON.stringify(value));
export const wrapAngle = value => Math.atan2(Math.sin(value), Math.cos(value));
export function finite(value, label) {
  if (!Number.isFinite(value)) throw new Error(`${label} must be a finite number`);
  return value;
}
