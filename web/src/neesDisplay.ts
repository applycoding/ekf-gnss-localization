// SPDX-License-Identifier: Apache-2.0
/** Position NEES reference band (PLAN matched/urban, bias model off). */
export const POS_NEES_LOW = 0.050636;
export const POS_NEES_HIGH = 7.377759;

export function neesSummaryLabel(scenario: string, meanNe: number): string {
  if (scenario === "urban_white_model") {
    return "diagnostic: model mismatch";
  }
  const meanText = Number.isFinite(meanNe) ? meanNe.toFixed(2) : "n/a";
  return `NEES mean ${meanText} (ref ${POS_NEES_LOW.toFixed(2)}–${POS_NEES_HIGH.toFixed(2)})`;
}
