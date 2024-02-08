import xarray
from scipy.ndimage import gaussian_filter


class Environment:
    """Contains the importance field"""

    def __init__(self, width: int, height: int, scenario: int) -> None:
        assert isinstance(width, int)
        assert isinstance(height, int)
        assert isinstance(scenario, int)
        self.importance_field = self.create_importance_field(width, height, scenario)

    def create_importance_field(
        self,
        width: int,
        height: int,
        scenario: int,
    ) -> xarray.DataArray:
        """Create importance field

        - `width` is raster coordinates
        - `height` is raster coordinates
        """
        assert isinstance(width, int)
        assert isinstance(height, int)
        assert isinstance(scenario, int)
        dims = {"x": width, "y": height}
        coords = {"x": range(width), "y": range(height)}
        Z = xarray.DataArray(data=0.0, dims=dims, coords=coords)
        match scenario:
            case 0:
                # Unipolar scenario
                x = int(0.5 * width)
                y = int(0.5 * height)
                Z[x, y] = 1.0
            case 1:
                # Unipolar scenario
                x = int(0.1 * width)
                y = int(0.1 * height)
                Z[x, y] = 1.0
            case 2:
                # Unipolar scenario
                x = int(0.1 * width)
                y = int(0.9 * height)
                Z[x, y] = 1.0
            case 3:
                # Bipolar scenario
                x = int(0.2 * width)
                y = int(0.2 * height)
                Z[x, y] = 1.0
                x = int(0.8 * width)
                y = int(0.8 * height)
                Z[x, y] = 1.0
            case other:
                print(f"unknown scenario: {other}")
                print(f"using default scenario parameters")
                # Use default parameters
                x = int(0.6 * width)
                y = int(0.6 * height)
                Z[x, y] = 1.0
                x = int(0.3 * width)
                y = int(0.5 * height)
                Z[x, y] = 1.0

        Z.data = gaussian_filter(Z, sigma=20)
        Z *= 1.0 / Z.max()

        # Extract target regions
        # threshold = 0.6
        # Z[Z < threshold] = 0

        # Assign uniform importance to each point
        # Z[0 < Z] = 1

        return Z
