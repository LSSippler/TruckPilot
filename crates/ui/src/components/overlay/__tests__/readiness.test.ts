import { describe, expect, it } from "vitest";
import { bbReadyTri } from "@/components/overlay/overlay-lib";

describe("bbReadyTri", () => {
  it("maps blackboard readiness strings", () => {
    expect(bbReadyTri("true")).toBe("yes");
    expect(bbReadyTri("false")).toBe("no");
    expect(bbReadyTri(undefined)).toBe("unknown");
    expect(bbReadyTri("")).toBe("unknown");
  });
});
