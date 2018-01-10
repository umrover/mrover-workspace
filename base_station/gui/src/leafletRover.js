import L from 'leaflet';

// save these original methods before they are overwritten
const proto_initIcon = L.Marker.prototype._initIcon;
const proto_setPos = L.Marker.prototype._setPos;

const oldIE = (L.DomUtil.TRANSFORM === 'msTransform');

L.Marker.addInitHook(function () {
  const iconOptions = this.options.icon && this.options.icon.options;
  let iconAnchor = iconOptions && this.options.icon.options.iconAnchor;
  if (iconAnchor) {
    iconAnchor = (`${iconAnchor[0]}px ${iconAnchor[1]}px`);
  }
  this.options.rotationOrigin = this.options.rotationOrigin || iconAnchor ||
    'center bottom';
  this.options.rotationAngle = this.options.rotationAngle || 0;

  // Ensure marker keeps rotated during dragging
  this.on('drag', (e) => {
    e.target._applyRotation();
  });
});

L.Marker.include({
  _initIcon() {
    proto_initIcon.call(this);
  },

  _setPos(pos) {
    proto_setPos.call(this, pos);
    this._applyRotation();
  },

  _applyRotation() {
    if (this.options.rotationAngle) {
      this._icon.style[`${L.DomUtil.TRANSFORM}Origin`] =
        this.options.rotationOrigin;

      if (oldIE) {
        // for IE 9, use the 2D rotation
        this._icon.style[L.DomUtil.TRANSFORM] =
          `rotate(${this.options.rotationAngle}deg)`;
      } else {
        // for modern browsers, prefer the 3D accelerated version
        this._icon.style[L.DomUtil.TRANSFORM] +=
          ` rotateZ(${this.options.rotationAngle}deg)`;
      }
    }
  },

  setRotationAngle(angle) {
    this.options.rotationAngle = angle;
    this.update();
    return this;
  },

  setRotationOrigin(origin) {
    this.options.rotationOrigin = origin;
    this.update();
    return this;
  },
});

export default L;
