#ifdef SHOW_UNWRAPPED_CLASSES

  %define WRAPPED_CLASS(Class...)
    %typemap(in) Class {}
    %typemap(out) Class {}
    %typemap(in) Class* {}
    %typemap(out) Class* {}
    %typemap(in) Class& {}
    %typemap(out) Class& {}
    %typemap(in) const Class {}
    %typemap(out) const Class {}
    %typemap(in) const Class* {}
    %typemap(out) const Class* {}
    %typemap(in) const Class& {}
    %typemap(out) const Class& {}
  %enddef

  %include SHOW_UNWRAPPED_CLASSES

  %typemap(in,  warning="901:$1_type") SWIGTYPE, SWIGTYPE*, SWIGTYPE&, const SWIGTYPE, const SWIGTYPE*, const SWIGTYPE& {}
  %typemap(out, warning="901:$1_type") SWIGTYPE, SWIGTYPE*, SWIGTYPE&, const SWIGTYPE, const SWIGTYPE*, const SWIGTYPE& {}

#endif
