######################################################################
#                                                                    #
#   DSO Recording script                                             #
#                                                                    #
######################################################################

#
# Inputs
#

# Required
#   Objects (positions)
#   Number of frames --> calculate required time
#   Image dest. paths

# Optional
#   HFDmax
#   



#
# Pre-Imaging
#

# Enable image camera cooling (if any)

# Montierung einnorden

# Grob Fokusieren

# Lightframes aufnehmen

# Light-Darkframes aufnehmen  --> later?

# Star-Alignment



#
# Imaging
#

# For each object i

   # Go to object i

   # Genau Fokusieren

   # Initialize guiding

      # Set focusing filter (if any)

      # Select guiding star

      # Calibrate guiding


   # For num light frames

      # Set required filter (LRGB)

      # Lightframes aufnehmen

      # Measure current HFD (do multiple measurements)

      # If HFD > HFDmax

         # Stop guiding

         # Set focusing filter (if any)

         # Re-Focusing

         # Start guiding
 
         # Wait for good guiding

      # End if

   # End for

   # Guiding stoppen

   # Zugehörige Darkframes aufnehmen

# End for



#
# Post-Imaging
#

# Disable image camera cooling (if any)

   # Wait until warm-up finished (?) 

# Montierung in Home-Pos fahren
