import { useEffect, useMemo } from "react";
import { Controller, useFieldArray, useForm, type Control, type FieldValues, type Path } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { jsonSchemaToZod } from "json-schema-to-zod";
import { z, type ZodTypeAny } from "zod";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Switch } from "@/components/ui/switch";
import { Slider } from "@/components/ui/slider";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";

interface JsonSchema {
  type?: string | string[];
  title?: string;
  description?: string;
  properties?: Record<string, JsonSchema>;
  required?: string[];
  enum?: unknown[];
  items?: JsonSchema;
  minimum?: number;
  maximum?: number;
  multipleOf?: number;
  format?: string;
  default?: unknown;
}

interface SchemaFormProps {
  schema: unknown;
  initialValues?: Record<string, unknown>;
  onSubmit: (values: Record<string, unknown>) => void | Promise<void>;
}

function buildZodFromSchema(schema: unknown): ZodTypeAny {
  try {
    const code = jsonSchemaToZod(schema as Parameters<typeof jsonSchemaToZod>[0], { module: "none" });
    const compiled = new Function("z", `return (${code});`)(z);
    return compiled as ZodTypeAny;
  } catch (err) {
    console.warn("[schema-form] failed to compile schema, falling back to passthrough", err);
    return z.record(z.string(), z.unknown());
  }
}

export function SchemaForm({ schema, initialValues, onSubmit }: SchemaFormProps) {
  const root = useMemo(() => (schema as JsonSchema | null) ?? { type: "object" as const }, [schema]);
  const zodSchema = useMemo(() => buildZodFromSchema(schema), [schema]);
  const defaults = useMemo(() => initialValues ?? extractDefaults(root) ?? {}, [initialValues, root]);

  const form = useForm<FieldValues>({
    resolver: zodResolver(zodSchema),
    defaultValues: defaults,
  });

  useEffect(() => {
    form.reset(defaults);
  }, [defaults, form]);

  const handleSubmit = form.handleSubmit(async (values) => {
    await onSubmit(values as Record<string, unknown>);
  });

  return (
    <form onSubmit={handleSubmit} className="space-y-4">
      <ObjectFields schema={root} control={form.control} pathPrefix="" />
      <div className="flex justify-end gap-2">
        <Button type="button" variant="outline" onClick={() => form.reset(defaults)}>
          Reset
        </Button>
        <Button type="submit" disabled={form.formState.isSubmitting}>
          Save
        </Button>
      </div>
    </form>
  );
}

function extractDefaults(schema: JsonSchema): Record<string, unknown> {
  if (schema.type !== "object" || !schema.properties) return {};
  const result: Record<string, unknown> = {};
  for (const [key, sub] of Object.entries(schema.properties)) {
    if (sub.default !== undefined) result[key] = sub.default;
    else if (sub.type === "object") result[key] = extractDefaults(sub);
    else if (sub.type === "array") result[key] = [];
    else if (sub.type === "boolean") result[key] = false;
    else if (sub.type === "number" || sub.type === "integer") result[key] = sub.minimum ?? 0;
    else result[key] = "";
  }
  return result;
}

interface FieldArgs {
  schema: JsonSchema;
  control: Control<FieldValues>;
  pathPrefix: string;
}

function ObjectFields({ schema, control, pathPrefix }: FieldArgs) {
  if (schema.type !== "object" || !schema.properties) return null;
  return (
    <div className="space-y-3">
      {Object.entries(schema.properties).map(([key, sub]) => {
        const path = pathPrefix ? `${pathPrefix}.${key}` : key;
        return <FieldRenderer key={path} name={key} schema={sub} control={control} pathPrefix={path} />;
      })}
    </div>
  );
}

function FieldRenderer({
  name,
  schema,
  control,
  pathPrefix,
}: FieldArgs & { name: string }) {
  const label = schema.title ?? humanise(name);
  const description = schema.description;
  const fieldName = pathPrefix as Path<FieldValues>;

  if (Array.isArray(schema.type) ? schema.type.includes("object") : schema.type === "object") {
    return (
      <div className="rounded-md border p-3">
        <div className="mb-2 text-sm font-semibold">{label}</div>
        {description ? <p className="mb-2 text-xs text-muted-foreground">{description}</p> : null}
        <ObjectFields schema={schema} control={control} pathPrefix={pathPrefix} />
      </div>
    );
  }

  if (schema.type === "array") {
    return <ArrayField label={label} description={description} schema={schema} control={control} pathPrefix={pathPrefix} />;
  }

  if (schema.enum && Array.isArray(schema.enum)) {
    return (
      <Field label={label} description={description}>
        <Controller
          name={fieldName}
          control={control}
          render={({ field }) => (
            <Select value={String(field.value ?? "")} onValueChange={(v) => field.onChange(v)}>
              <SelectTrigger>
                <SelectValue placeholder="Select…" />
              </SelectTrigger>
              <SelectContent>
                {schema.enum?.map((opt) => (
                  <SelectItem key={String(opt)} value={String(opt)}>
                    {String(opt)}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          )}
        />
      </Field>
    );
  }

  if (schema.type === "boolean") {
    return (
      <Field label={label} description={description} inline>
        <Controller
          name={fieldName}
          control={control}
          render={({ field }) => (
            <Switch checked={Boolean(field.value)} onCheckedChange={(v) => field.onChange(Boolean(v))} />
          )}
        />
      </Field>
    );
  }

  if (schema.type === "number" || schema.type === "integer") {
    const min = schema.minimum;
    const max = schema.maximum;
    if (typeof min === "number" && typeof max === "number") {
      const step = schema.multipleOf ?? (schema.type === "integer" ? 1 : (max - min) / 100);
      return (
        <Field label={label} description={description}>
          <Controller
            name={fieldName}
            control={control}
            render={({ field }) => (
              <div className="flex items-center gap-3">
                <Slider
                  value={[Number(field.value ?? min)]}
                  min={min}
                  max={max}
                  step={step}
                  onValueChange={(vals) => field.onChange(vals[0])}
                  className="flex-1"
                />
                <span className="w-16 text-right font-mono text-sm">
                  {typeof field.value === "number" ? field.value.toFixed(schema.type === "integer" ? 0 : 2) : "0"}
                </span>
              </div>
            )}
          />
        </Field>
      );
    }
    return (
      <Field label={label} description={description}>
        <Controller
          name={fieldName}
          control={control}
          render={({ field }) => (
            <Input
              type="number"
              value={field.value ?? ""}
              step={schema.multipleOf ?? "any"}
              onChange={(e) => field.onChange(e.target.value === "" ? null : Number(e.target.value))}
            />
          )}
        />
      </Field>
    );
  }

  if (schema.format === "color") {
    return (
      <Field label={label} description={description}>
        <Controller
          name={fieldName}
          control={control}
          render={({ field }) => (
            <Input type="color" value={String(field.value ?? "#000000")} onChange={(e) => field.onChange(e.target.value)} />
          )}
        />
      </Field>
    );
  }

  return (
    <Field label={label} description={description}>
      <Controller
        name={fieldName}
        control={control}
        render={({ field }) => (
          <Input
            value={field.value == null ? "" : String(field.value)}
            onChange={(e) => field.onChange(e.target.value)}
          />
        )}
      />
    </Field>
  );
}

function ArrayField({
  label,
  description,
  schema,
  control,
  pathPrefix,
}: FieldArgs & { label: string; description?: string }) {
  const { fields, append, remove } = useFieldArray({
    control,
    name: pathPrefix as Path<FieldValues>,
  });
  const itemSchema = schema.items ?? { type: "string" };

  return (
    <div className="rounded-md border p-3">
      <div className="mb-2 flex items-center justify-between">
        <div>
          <div className="text-sm font-semibold">{label}</div>
          {description ? <p className="text-xs text-muted-foreground">{description}</p> : null}
        </div>
        <Button type="button" size="sm" variant="outline" onClick={() => append(itemDefault(itemSchema))}>
          Add
        </Button>
      </div>
      <ul className="space-y-2">
        {fields.map((entry, idx) => (
          <li key={entry.id} className="flex items-start gap-2">
            <div className="flex-1">
              <FieldRenderer
                name={`item ${idx + 1}`}
                schema={itemSchema}
                control={control}
                pathPrefix={`${pathPrefix}.${idx}`}
              />
            </div>
            <Button type="button" size="sm" variant="outline" onClick={() => remove(idx)}>
              Remove
            </Button>
          </li>
        ))}
      </ul>
    </div>
  );
}

function itemDefault(schema: JsonSchema): unknown {
  if (schema.default !== undefined) return schema.default;
  if (schema.type === "object") return extractDefaults(schema);
  if (schema.type === "array") return [];
  if (schema.type === "boolean") return false;
  if (schema.type === "number" || schema.type === "integer") return schema.minimum ?? 0;
  return "";
}

function Field({
  label,
  description,
  inline,
  children,
}: {
  label: string;
  description?: string;
  inline?: boolean;
  children: React.ReactNode;
}) {
  return (
    <div className={inline ? "flex items-center justify-between gap-3" : "space-y-1.5"}>
      <div>
        <Label className="text-sm">{label}</Label>
        {description ? <p className="text-xs text-muted-foreground">{description}</p> : null}
      </div>
      <div className={inline ? "" : "w-full"}>{children}</div>
    </div>
  );
}

function humanise(key: string): string {
  return key.replace(/_/g, " ").replace(/\b\w/g, (c) => c.toUpperCase());
}
