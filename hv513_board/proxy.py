from path_helpers import path
try:
    from base_node_rpc.proxy import ConfigMixinBase, StateMixinBase
    from .node import (Proxy as _Proxy, I2cProxy as _I2cProxy,
                       SerialProxy as _SerialProxy)
    from .config import Config
    from .state import State


    class ConfigMixin(ConfigMixinBase):
        @property
        def config_class(self):
            return Config


    class StateMixin(StateMixinBase):
        @property
        def state_class(self):
            return State


    class ProxyMixin(ConfigMixin, StateMixin):
        '''
        Mixin class to add convenience wrappers around methods of the generated
        `node.Proxy` class.
        '''
        host_package_name = str(path(__file__).parent.name.replace('_', '-'))

        @property
        def frequency(self):
            return self.state['frequency']
        
        @frequency.setter
        def frequency(self, value):
            return self.update_state(frequency=value)

        @property
        def voltage(self):
            return self.state['voltage']

        @voltage.setter
        def voltage(self, value):
            return self.update_state(voltage=value)

        @property
        def output_enabled(self):
            return self.state['output_enabled']

        @output_enabled.setter
        def output_enabled(self, value):
            return self.update_state(output_enabled=value)

        def _state_of_channels(self):
            '''
            Prepend underscore to the auto-generated state_of_channels accessor
            '''
            return super(ProxyMixin, self).state_of_channels()

        def _set_state_of_channels(self, states):
            '''
            Prepend underscore to the auto-generated state_of_channels setter
            '''
            return super(ProxyMixin, self).set_state_of_channels(states)

        @property
        def state_of_channels(self):
            '''
            Retrieve the state bytes from the device and unpacks them into an
            array with one entry per channel.  Return unpacked array.

            Notes
            -----

            State of each channel is binary, 0 or 1.  On device, states are
            stored in bytes, where each byte corresponds to the state of eight
            channels.
            '''
            import numpy as np

            return np.unpackbits(super(ProxyMixin, self).state_of_channels())

        @state_of_channels.setter
        def state_of_channels(self, states):
            self.set_state_of_channels(states)

        def set_state_of_channels(self, states):
            '''
            Pack array containing one entry per channel to bytes (8 channels
            per byte).  Set state of channels on device using state bytes.

            See also: `state_of_channels` (get)
            '''
            import numpy as np
            
            ok = (super(ProxyMixin, self)
                    .set_state_of_channels(np.packbits(states.astype(int))))
            if not ok:
                raise ValueError('Error setting state of channels.  Check '
                                 'number of states matches channel count.')
                
        @property
        def baud_rate(self):
            return self.config['baud_rate']

        @baud_rate.setter
        def baud_rate(self, baud_rate):
            return self.update_config(baud_rate=baud_rate)

        @property
        def serial_number(self):
            return self.config['serial_number']

        @serial_number.setter
        def serial_number(self, serial_number):
            return self.update_config(serial_number=serial_number)
        
        @property
        def port(self):
            return self._stream.serial_device.port

        @port.setter
        def port(self, port):
            return self.update_config(port=port)
        
        def _number_of_channels(self):
            return super(ProxyMixin, self).number_of_channels()
        
        @property
        def number_of_channels(self):
            return self._number_of_channels()
        
        @property
        def hardware_version(self):
            '''
            Placeholder for a remote hardware_version accessor
            '''
            return '0.1'

        @property
        def min_waveform_frequency(self):
            return self.config['min_waveform_frequency']

        @min_waveform_frequency.setter
        def min_waveform_frequency(self, min_waveform_frequency):
            return self.update_config(min_waveform_frequency=min_waveform_frequency)
        
        @property
        def max_waveform_frequency(self):
            return self.config['max_waveform_frequency']

        @max_waveform_frequency.setter
        def max_waveform_frequency(self, max_waveform_frequency):
            return self.update_config(max_waveform_frequency=max_waveform_frequency)

        @property
        def max_waveform_voltage(self):
            return self.config['max_waveform_voltage']

        @max_waveform_voltage.setter
        def max_waveform_voltage(self, max_waveform_voltage):
            return self.update_config(max_waveform_voltage=max_waveform_voltage)


    class Proxy(ProxyMixin, _Proxy):
        pass

    class I2cProxy(ProxyMixin, _I2cProxy):
        pass

    class SerialProxy(ProxyMixin, _SerialProxy):
        pass

except (ImportError, TypeError):
    Proxy = None
    I2cProxy = None
    SerialProxy = None
