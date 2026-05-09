import { useEffect, useRef } from "react";
import { subscribeToCoreEvents } from "@/lib/ipc";
import type { CoreMessageOf, CoreMessageType } from "@/lib/types";

export function useIpcEvent<T extends CoreMessageType>(
  type: T,
  handler: (msg: CoreMessageOf<T>) => void
) {
  const handlerRef = useRef(handler);
  handlerRef.current = handler;

  useEffect(() => {
    return subscribeToCoreEvents(type, (msg) => handlerRef.current(msg));
  }, [type]);
}
