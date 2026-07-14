(function () {
  const defaultCenter = [45.5350, 11.5455];

  const map = L.map("map", {
    zoomControl: true,
    attributionControl: true,
    maxZoom: 19
  }).setView(defaultCenter, 12);

  const osmLayer = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "&copy; OpenStreetMap contributors"
  });

  const imageryLayer = L.tileLayer(
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    {
      maxZoom: 19,
      attribution: "Tiles &copy; Esri"
    }
  );

  const LocationControl = L.Control.extend({
    options: {
      position: "topleft"
    },

    onAdd: function () {
      const container = L.DomUtil.create("div", "leaflet-bar leaflet-control leaflet-control-location");
      const link = L.DomUtil.create("a", "leaflet-control-location-button", container);
      link.href = "#";
      link.title = "Center on GPS position";
      link.setAttribute("aria-label", "Center on GPS position");
      link.innerHTML = '<span class="location-crosshair" aria-hidden="true">&#8982;</span>';

      L.DomEvent.disableClickPropagation(container);
      L.DomEvent.on(link, "click", L.DomEvent.stop);
      L.DomEvent.on(link, "click", function () {
        if (window.AndroidBridge && window.AndroidBridge.requestUserLocation) {
          window.AndroidBridge.requestUserLocation();
        }
      });

      return container;
    }
  });

  map.addControl(new LocationControl());

  const polygonLayer = L.layerGroup().addTo(map);
  const vertexLayer = L.layerGroup().addTo(map);
  const previewLayer = L.layerGroup().addTo(map);
  const takeoffLayer = L.layerGroup().addTo(map);
  const locationLayer = L.layerGroup().addTo(map);
  const rotationLayer = L.layerGroup().addTo(map);

  const vertexIcon = L.divIcon({
    className: "vertex-marker",
    html: '<div class="vertex-dot"></div>',
    iconSize: [18, 18],
    iconAnchor: [9, 9]
  });

  const takeoffIcon = L.divIcon({
    className: "takeoff-marker",
    html: '<div class="takeoff-dot"></div>',
    iconSize: [24, 24],
    iconAnchor: [12, 12]
  });

  const routeStartIcon = L.divIcon({
    className: "route-label-marker",
    html: '<div class="route-label route-label-start">START</div>',
    iconSize: [58, 24],
    iconAnchor: [29, 12]
  });

  const routeEndIcon = L.divIcon({
    className: "route-label-marker",
    html: '<div class="route-label route-label-end">END</div>',
    iconSize: [46, 24],
    iconAnchor: [23, 12]
  });

  const state = {
    rawPoints: [],
    polygon: [],
    takeoff: null,
    waypoints: [],
    mode: "polygon",
    baseMap: "imagery",
    gridAngleDeg: 0,
    showGridRotationHandle: false
  };

  let polygonShape = null;
  let rotationCenterMarker = null;
  let rotationHandleMarker = null;
  let rotationGuide = null;
  let rotationHandleDragging = false;
  let rotationNotifyTimer = null;

  function clonePoint(latlng) {
    return {
      lat: Number(latlng.lat.toFixed(7)),
      lon: Number(latlng.lng.toFixed(7))
    };
  }

  function samePoint(a, b) {
    return Math.abs(a.lat - b.lat) < 1e-7 && Math.abs(a.lon - b.lon) < 1e-7;
  }

  function sanitizeOrderedPoints(points) {
    const normalized = [];
    points.forEach((point) => {
      const candidate = {
        lat: Number(point.lat),
        lon: Number(point.lon)
      };
      if (!normalized.length || !samePoint(normalized[normalized.length - 1], candidate)) {
        normalized.push(candidate);
      }
    });

    if (normalized.length >= 2 && samePoint(normalized[0], normalized[normalized.length - 1])) {
      normalized.pop();
    }

    return normalized;
  }

  function signedArea(points) {
    if (points.length < 3) {
      return 0;
    }

    let area = 0;
    for (let index = 0; index < points.length; index += 1) {
      const current = points[index];
      const next = points[(index + 1) % points.length];
      area += (current.lon * next.lat) - (next.lon * current.lat);
    }
    return area / 2;
  }

  function orientation(a, b, c) {
    const value = (b.lon - a.lon) * (c.lat - a.lat) - (b.lat - a.lat) * (c.lon - a.lon);
    if (Math.abs(value) < 1e-10) {
      return 0;
    }
    return value > 0 ? 1 : -1;
  }

  function onSegment(a, b, point) {
    return (
      point.lon <= Math.max(a.lon, b.lon) + 1e-10 &&
      point.lon >= Math.min(a.lon, b.lon) - 1e-10 &&
      point.lat <= Math.max(a.lat, b.lat) + 1e-10 &&
      point.lat >= Math.min(a.lat, b.lat) - 1e-10
    );
  }

  function segmentsIntersect(aStart, aEnd, bStart, bEnd) {
    const o1 = orientation(aStart, aEnd, bStart);
    const o2 = orientation(aStart, aEnd, bEnd);
    const o3 = orientation(bStart, bEnd, aStart);
    const o4 = orientation(bStart, bEnd, aEnd);

    if (o1 !== o2 && o3 !== o4) {
      return true;
    }

    if (o1 === 0 && onSegment(aStart, aEnd, bStart)) return true;
    if (o2 === 0 && onSegment(aStart, aEnd, bEnd)) return true;
    if (o3 === 0 && onSegment(bStart, bEnd, aStart)) return true;
    if (o4 === 0 && onSegment(bStart, bEnd, aEnd)) return true;
    return false;
  }

  function polygonHasSelfIntersection(points) {
    const normalized = sanitizeOrderedPoints(points);
    if (normalized.length < 4) {
      return false;
    }

    const edgeCount = normalized.length;
    for (let firstIndex = 0; firstIndex < edgeCount; firstIndex += 1) {
      const firstNext = (firstIndex + 1) % edgeCount;
      for (let secondIndex = firstIndex + 1; secondIndex < edgeCount; secondIndex += 1) {
        const secondNext = (secondIndex + 1) % edgeCount;
        const sharesVertex =
          firstIndex === secondIndex ||
          firstIndex === secondNext ||
          firstNext === secondIndex ||
          firstNext === secondNext;
        if (sharesVertex) {
          continue;
        }

        if (
          segmentsIntersect(
            normalized[firstIndex],
            normalized[firstNext],
            normalized[secondIndex],
            normalized[secondNext]
          )
        ) {
          return true;
        }
      }
    }

    return false;
  }

  function applyOrderedPolygon(points) {
    let normalized = sanitizeOrderedPoints(points);
    if (normalized.length >= 3 && signedArea(normalized) < 0) {
      normalized = normalized.slice().reverse();
    }

    state.rawPoints = normalized.slice();
    state.polygon = normalized.slice();
  }

  function notifyPolygonInvalid() {
    if (window.AndroidBridge && window.AndroidBridge.onPolygonInvalid) {
      window.AndroidBridge.onPolygonInvalid();
    }
  }

  function tryUpdatePolygon(points, notifyChange) {
    const normalized = sanitizeOrderedPoints(points);
    if (polygonHasSelfIntersection(normalized)) {
      notifyPolygonInvalid();
      return false;
    }

    applyOrderedPolygon(normalized);
    if (notifyChange) {
      notifyPolygonChanged();
    }
    return true;
  }

  function pointDistance(a, b) {
    return map.distance(L.latLng(a.lat, a.lon), L.latLng(b.lat, b.lon));
  }

  function insertedPolygon(points, insertAfterIndex, point) {
    return points
      .slice(0, insertAfterIndex + 1)
      .concat([{ lat: point.lat, lon: point.lon }], points.slice(insertAfterIndex + 1));
  }

  function bestInsertionForPoint(points, point) {
    const normalized = sanitizeOrderedPoints(points);
    if (!normalized.length) {
      return [point];
    }

    if (normalized.some((candidate) => samePoint(candidate, point))) {
      return normalized;
    }

    if (normalized.length < 3) {
      return normalized.concat([{ lat: point.lat, lon: point.lon }]);
    }

    let bestCandidate = null;
    let bestScore = Number.POSITIVE_INFINITY;

    for (let index = 0; index < normalized.length; index += 1) {
      const nextIndex = (index + 1) % normalized.length;
      const start = normalized[index];
      const end = normalized[nextIndex];
      const candidate = nextIndex === 0
        ? [{ lat: point.lat, lon: point.lon }].concat(normalized)
        : insertedPolygon(normalized, index, point);

      if (polygonHasSelfIntersection(candidate)) {
        continue;
      }

      const addedLength =
        pointDistance(start, point) +
        pointDistance(point, end) -
        pointDistance(start, end);

      if (addedLength < bestScore) {
        bestScore = addedLength;
        bestCandidate = candidate;
      }
    }

    return bestCandidate;
  }

  function formatPoint(point) {
    return point.lat.toFixed(6) + ", " + point.lon.toFixed(6);
  }

  function formatElevation(point) {
    return typeof point.elevationM === "number"
      ? point.elevationM.toFixed(1) + " m"
      : "n/a";
  }

  function normalizeAngle(angleDeg) {
    return ((angleDeg % 360) + 360) % 360;
  }

  function gridAngleToBearing(angleDeg) {
    return normalizeAngle(90 - angleDeg);
  }

  function bearingToGridAngle(bearingDeg) {
    return normalizeAngle(90 - bearingDeg);
  }

  function averagePoint(points) {
    if (!points.length) {
      return null;
    }

    const sums = points.reduce((acc, point) => {
      acc.lat += point.lat;
      acc.lon += point.lon;
      return acc;
    }, { lat: 0, lon: 0 });

    return L.latLng(sums.lat / points.length, sums.lon / points.length);
  }

  function bearingDegrees(from, to) {
    return normalizeAngle(Math.atan2(to.lng - from.lng, to.lat - from.lat) * 180 / Math.PI);
  }

  function rotationCenter() {
    return averagePoint(state.polygon);
  }

  function centerRotationIcon(angleDeg) {
    const bearingDeg = gridAngleToBearing(angleDeg);
    return L.divIcon({
      className: "grid-rotation-marker",
      html:
        '<div class="grid-rotation-core">' +
        '<div class="grid-rotation-arrow" style="transform: rotate(' + bearingDeg.toFixed(1) + 'deg)"></div>' +
        "</div>",
      iconSize: [48, 48],
      iconAnchor: [24, 24]
    });
  }

  function rotationHandleIcon() {
    return L.divIcon({
      className: "grid-rotation-handle-marker",
      html: '<div class="grid-rotation-handle"></div>',
      iconSize: [24, 24],
      iconAnchor: [12, 12]
    });
  }

  function scheduleGridAngleNotify(angleDeg, immediate) {
    const snappedAngle = Math.round(normalizeAngle(angleDeg));
    if (rotationNotifyTimer) {
      window.clearTimeout(rotationNotifyTimer);
      rotationNotifyTimer = null;
    }

    const notify = function () {
      if (window.AndroidBridge && window.AndroidBridge.onGridAngleChanged) {
        window.AndroidBridge.onGridAngleChanged(snappedAngle);
      }
    };

    if (immediate) {
      notify();
      return;
    }

    rotationNotifyTimer = window.setTimeout(notify, 80);
  }

  function clearRotationHandle() {
    map.dragging.enable();
    rotationLayer.clearLayers();
    rotationCenterMarker = null;
    rotationHandleMarker = null;
    rotationGuide = null;
    rotationHandleDragging = false;
  }

  function updateRotationGuide(center, handle, visible) {
    if (!rotationGuide) {
      return;
    }

    rotationGuide.setLatLngs([center, handle]);
    rotationGuide.setStyle({
      opacity: visible ? 0.85 : 0
    });
  }

  function updateRotationPreview(center) {
    if (!rotationCenterMarker) {
      return;
    }

    rotationCenterMarker.setLatLng(center);
    rotationCenterMarker.setIcon(centerRotationIcon(state.gridAngleDeg));
    if (rotationHandleMarker && !rotationHandleDragging) {
      rotationHandleMarker.setLatLng(center);
      updateRotationGuide(center, center, false);
    }
  }

  function rebuildRotationHandle() {
    if (!state.showGridRotationHandle || state.polygon.length < 3) {
      clearRotationHandle();
      return;
    }

    const center = rotationCenter();

    if (!rotationCenterMarker) {
      rotationGuide = L.polyline([center, center], {
        color: "#114b9b",
        weight: 2,
        opacity: 0,
        dashArray: "4 6",
        interactive: false
      }).addTo(rotationLayer);

      rotationCenterMarker = L.marker(center, {
        icon: centerRotationIcon(state.gridAngleDeg),
        interactive: false,
        keyboard: false,
        bubblingMouseEvents: false
      }).addTo(rotationLayer);

      rotationHandleMarker = L.marker(center, {
        icon: rotationHandleIcon(),
        draggable: true,
        keyboard: false,
        bubblingMouseEvents: false,
        zIndexOffset: 1000
      }).addTo(rotationLayer);

      rotationHandleMarker.on("dragstart", function (event) {
        rotationHandleDragging = true;
        map.dragging.disable();
        const activeCenter = rotationCenter();
        event.target.setLatLng(activeCenter);
        updateRotationGuide(activeCenter, activeCenter, true);
      });

      rotationHandleMarker.on("drag", function (event) {
        const activeCenter = rotationCenter();
        const dragLatLng = event.target.getLatLng();
        updateRotationGuide(activeCenter, dragLatLng, true);
        if (map.distance(activeCenter, dragLatLng) < 0.5) {
          return;
        }
        const dragBearing = bearingDegrees(activeCenter, dragLatLng);
        state.gridAngleDeg = bearingToGridAngle(dragBearing);
        rotationCenterMarker.setIcon(centerRotationIcon(state.gridAngleDeg));
      });

      rotationHandleMarker.on("dragend", function (event) {
        const activeCenter = rotationCenter();
        const dragLatLng = event.target.getLatLng();
        if (map.distance(activeCenter, dragLatLng) >= 0.5) {
          const dragBearing = bearingDegrees(activeCenter, dragLatLng);
          state.gridAngleDeg = bearingToGridAngle(dragBearing);
        }
        event.target.setLatLng(activeCenter);
        rotationCenterMarker.setLatLng(activeCenter);
        rotationCenterMarker.setIcon(centerRotationIcon(state.gridAngleDeg));
        updateRotationGuide(activeCenter, activeCenter, false);
        rotationHandleDragging = false;
        rebuildRotationHandle();
        map.dragging.enable();
        scheduleGridAngleNotify(state.gridAngleDeg, true);
      });
    }

    updateRotationPreview(center);
  }

  function notifyPolygonChanged() {
    if (window.AndroidBridge && window.AndroidBridge.onPolygonChanged) {
      window.AndroidBridge.onPolygonChanged(JSON.stringify(state.rawPoints));
    }
  }

  function notifyTakeoffChanged() {
    if (window.AndroidBridge && window.AndroidBridge.onTakeoffPointChanged) {
      if (state.takeoff) {
        window.AndroidBridge.onTakeoffPointChanged(JSON.stringify(state.takeoff));
      } else {
        window.AndroidBridge.onTakeoffPointChanged(JSON.stringify({ clear: true }));
      }
    }
  }

  function rebuildPolygonShape() {
    polygonLayer.clearLayers();

    if (state.polygon.length >= 2) {
      const latlngs = state.polygon.map((point) => [point.lat, point.lon]);
      polygonShape = state.polygon.length >= 3
        ? L.polygon(latlngs, {
            color: "#1f4b43",
            fillColor: "#1f4b43",
            fillOpacity: 0.16,
            weight: 3
          }).addTo(polygonLayer)
        : L.polyline(latlngs, {
            color: "#1f4b43",
            weight: 3
          }).addTo(polygonLayer);
    } else {
      polygonShape = null;
    }
  }

  function rebuildPolygonGraphics() {
    vertexLayer.clearLayers();
    rebuildPolygonShape();

    state.rawPoints.forEach((point, index) => {
      const marker = L.marker([point.lat, point.lon], {
        draggable: true,
        icon: vertexIcon,
        keyboard: false
      }).addTo(vertexLayer);

      marker.bindPopup(
        "Point " + (index + 1) +
        "<br>" + formatPoint(point)
      );

      marker.on("drag", function (event) {
        const candidatePoints = state.rawPoints.slice();
        candidatePoints[index] = clonePoint(event.target.getLatLng());
        if (tryUpdatePolygon(candidatePoints, false)) {
          rebuildPolygonShape();
        }
      });

      marker.on("dragend", function (event) {
        const candidatePoints = state.rawPoints.slice();
        candidatePoints[index] = clonePoint(event.target.getLatLng());
        if (!tryUpdatePolygon(candidatePoints, true)) {
          marker.setLatLng([point.lat, point.lon]);
        }
        rebuildPolygonGraphics();
      });

      const removeVertex = function () {
        const candidatePoints = state.rawPoints.slice();
        candidatePoints.splice(index, 1);
        tryUpdatePolygon(candidatePoints, true);
        rebuildPolygonGraphics();
      };

      marker.on("dblclick", removeVertex);
      marker.on("contextmenu", removeVertex);
    });
  }

  function rebuildTakeoffMarker() {
    takeoffLayer.clearLayers();
    if (!state.takeoff) {
      return;
    }

    const marker = L.marker([state.takeoff.lat, state.takeoff.lon], {
      draggable: true,
      icon: takeoffIcon,
      keyboard: false
    }).addTo(takeoffLayer);

    marker.bindPopup(
      "Takeoff point" +
      "<br>" + formatPoint(state.takeoff) +
      "<br>ground: " + formatElevation(state.takeoff)
    );

    marker.on("drag", function (event) {
      state.takeoff = clonePoint(event.target.getLatLng());
    });

    marker.on("dragend", function (event) {
      state.takeoff = clonePoint(event.target.getLatLng());
      notifyTakeoffChanged();
    });
  }

  function rebuildWaypoints() {
    previewLayer.clearLayers();

    if (!state.waypoints || state.waypoints.length === 0) {
      return;
    }

    const latlngs = state.waypoints.map((waypoint) => [waypoint.lat, waypoint.lon]);
    L.polyline(latlngs, {
      color: "#b8691b",
      weight: 3,
      opacity: 0.95
    }).addTo(previewLayer);

    const firstWaypoint = state.waypoints[0];
    const lastWaypoint = state.waypoints[state.waypoints.length - 1];
    L.marker([firstWaypoint.lat, firstWaypoint.lon], {
      icon: routeStartIcon,
      keyboard: false
    }).addTo(previewLayer).bindPopup("Route start<br>" + formatPoint(firstWaypoint));

    if (!samePoint(firstWaypoint, lastWaypoint)) {
      L.marker([lastWaypoint.lat, lastWaypoint.lon], {
        icon: routeEndIcon,
        keyboard: false
      }).addTo(previewLayer).bindPopup("Route end<br>" + formatPoint(lastWaypoint));
    }
  }

  function rebuildUserLocation(lat, lon, accuracyM, focus) {
    locationLayer.clearLayers();

    L.circleMarker([lat, lon], {
      radius: 7,
      color: "#114b9b",
      weight: 3,
      fillColor: "#2e87ff",
      fillOpacity: 0.9
    }).addTo(locationLayer).bindPopup(
      "GPS position" +
      "<br>" + lat.toFixed(6) + ", " + lon.toFixed(6) +
      "<br>accuracy: " + Math.max(accuracyM || 0, 0).toFixed(1) + " m"
    );

    if ((accuracyM || 0) > 0) {
      L.circle([lat, lon], {
        radius: accuracyM,
        color: "#2e87ff",
        weight: 1,
        fillColor: "#2e87ff",
        fillOpacity: 0.12
      }).addTo(locationLayer);
    }

    if (focus) {
      map.setView([lat, lon], Math.min(19, Math.max(map.getZoom(), 17)));
    }
  }

  function focusProject(project) {
    if (state.polygon.length >= 2) {
      map.fitBounds(state.polygon.map((point) => [point.lat, point.lon]), {
        padding: [24, 24],
        maxZoom: 19
      });
      return;
    }

    if (state.waypoints && state.waypoints.length >= 2) {
      map.fitBounds(state.waypoints.map((waypoint) => [waypoint.lat, waypoint.lon]), {
        padding: [24, 24],
        maxZoom: 19
      });
      return;
    }

    if (state.takeoff) {
      map.setView([state.takeoff.lat, state.takeoff.lon], Math.min(19, Math.max(map.getZoom(), 17)));
      return;
    }

    if (project.center) {
      map.setView([project.center.lat, project.center.lon], map.getZoom());
    }
  }

  function renderProject(project) {
    const polygonPoints = Array.isArray(project.rawPolygonPoints)
      ? project.rawPolygonPoints.slice()
      : (Array.isArray(project.polygon) ? project.polygon.slice() : []);
    applyOrderedPolygon(polygonPoints);
    state.takeoff = project.takeoff && typeof project.takeoff.lat === "number" ? project.takeoff : null;
    state.waypoints = Array.isArray(project.waypoints) ? project.waypoints.slice() : [];
    state.gridAngleDeg = normalizeAngle(project.gridAngleDeg || 0);
    state.showGridRotationHandle = Boolean(project.showGridRotationHandle);

    rebuildPolygonGraphics();
    rebuildTakeoffMarker();
    rebuildWaypoints();
    rebuildRotationHandle();

    if (project.focusProject) {
      focusProject(project);
    }
  }

  function setMode(mode) {
    state.mode = mode === "takeoff" ? "takeoff" : "polygon";
  }

  function setBaseMap(baseMap) {
    state.baseMap = baseMap === "osm" ? "osm" : "imagery";
    map.removeLayer(osmLayer);
    map.removeLayer(imageryLayer);
    if (state.baseMap === "osm") {
      osmLayer.addTo(map);
    } else {
      imageryLayer.addTo(map);
    }
  }

  map.on("click", function (event) {
    if (state.mode === "takeoff") {
      state.takeoff = clonePoint(event.latlng);
      rebuildTakeoffMarker();
      notifyTakeoffChanged();
      setMode("polygon");
      return;
    }

    const clickedPoint = clonePoint(event.latlng);
    const candidate = bestInsertionForPoint(state.rawPoints, clickedPoint);
    if (!candidate) {
      notifyPolygonInvalid();
      return;
    }

    if (!tryUpdatePolygon(candidate, true)) {
      return;
    }
    rebuildPolygonGraphics();
  });

  window.plannerMap = {
    renderProject: renderProject,
    setMode: setMode,
    setBaseMap: setBaseMap,
    showUserLocation: rebuildUserLocation
  };

  setBaseMap("imagery");

  if (window.AndroidBridge && window.AndroidBridge.onMapReady) {
    window.AndroidBridge.onMapReady();
  }
})();
