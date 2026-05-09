import { describe, expect, it, vi } from "vitest";
import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { SchemaForm } from "@/components/schema-form/SchemaForm";

describe("SchemaForm", () => {
  it("renders fields per JSON schema and submits typed values", async () => {
    const schema = {
      type: "object",
      properties: {
        name: { type: "string", title: "Name", default: "TruckPilot" },
        speed_kmh: { type: "integer", title: "Speed", default: 80 },
        active: { type: "boolean", title: "Active", default: true },
      },
      required: ["name"],
    };
    const onSubmit = vi.fn().mockResolvedValue(undefined);

    render(<SchemaForm schema={schema} onSubmit={onSubmit} />);

    expect(screen.getByText("Name")).toBeInTheDocument();
    expect(screen.getByText("Speed")).toBeInTheDocument();
    expect(screen.getByText("Active")).toBeInTheDocument();

    const textInput = screen.getByDisplayValue("TruckPilot");
    fireEvent.change(textInput, { target: { value: "Pilot" } });
    fireEvent.click(screen.getByRole("button", { name: "Save" }));

    await waitFor(() => expect(onSubmit).toHaveBeenCalledTimes(1));
    expect(onSubmit.mock.calls[0]?.[0]).toMatchObject({
      name: "Pilot",
      speed_kmh: 80,
      active: true,
    });
  });

  it("renders a slider with formatted readout when min and max are present", () => {
    const schema = {
      type: "object",
      properties: {
        gain: { type: "number", minimum: 0, maximum: 1, default: 0.5, title: "Gain" },
      },
    };
    render(<SchemaForm schema={schema} onSubmit={vi.fn()} />);
    expect(screen.getByText("Gain")).toBeInTheDocument();
    expect(screen.getByText(/0\.50/)).toBeInTheDocument();
  });

  it("falls back to a passthrough schema when the provided schema is malformed", () => {
    const onSubmit = vi.fn();
    render(<SchemaForm schema={null} onSubmit={onSubmit} />);
    expect(screen.getByRole("button", { name: "Save" })).toBeInTheDocument();
  });
});
