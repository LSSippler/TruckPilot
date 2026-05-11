import { usePluginsStore } from "@/stores/plugins";

export function usePlugins() {
  return usePluginsStore((s) => ({
    list: s.list,
    order: s.order,
    selected: s.selected,
    select: s.select,
  }));
}
